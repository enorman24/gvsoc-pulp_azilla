/*
 * Copyright (C) 2020 GreenWaves Technologies, SAS, ETH Zurich and
 *                    University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * io_v2 port of the Spatz cluster peripheral registers. Same regmap and
 * behaviour as cluster_registers.cpp; the IO plumbing follows io_v2:
 *   - normal accesses answer inline with IO_REQ_DONE (+latency annotation)
 *   - a core loading HW_BARRIER before the barrier is reached gets
 *     IO_REQ_GRANTED; the response is sent through the core's own slave
 *     port once the last core reaches the barrier.
 */

#include <vector>
#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <vp/itf/wire.hpp>
#include <pulp/snitch/snitch_cluster/spatz/cluster_periph_regfields.h>
#include <pulp/snitch/snitch_cluster/spatz/cluster_periph_gvsoc.h>


using namespace std::placeholders;


class ClusterRegisters : public vp::Component
{

public:

    ClusterRegisters(vp::ComponentConf &config);

    void reset(bool active);


private:
    static vp::IoReqStatus req(vp::Block *__this, vp::IoReq *req);
    static vp::IoReqStatus core_req(vp::Block *__this, vp::IoReq *req, int id);
    void cl_clint_set_req(uint64_t reg_offset, int size, uint8_t *value, bool is_write);
    void cl_clint_clear_req(uint64_t reg_offset, int size, uint8_t *value, bool is_write);
    void hw_barrier_req(uint64_t reg_offset, int size, uint8_t *value, bool is_write);

    vp::Trace     trace;

    vp_regmap_cluster_periph regmap;

    vp::IoSlave in{&ClusterRegisters::req};
    std::vector<vp::IoSlave> cores_in;
    uint32_t bootaddr;
    int nb_cores;
    vp::reg_32 barrier_status;

    std::vector<vp::WireMaster<bool>> external_irq_itf;

    int core_access;
    bool stall_core;
    uint32_t waiting_cores;

    std::vector<vp::IoReq *> waiting_reqs;
};

ClusterRegisters::ClusterRegisters(vp::ComponentConf &config)
: vp::Component(config), regmap(*this, "regmap")
{
    this->traces.new_trace("trace", &trace, vp::DEBUG);

    this->bootaddr = this->get_js_config()->get("boot_addr")->get_int();
    this->nb_cores = this->get_js_config()->get("nb_cores")->get_int();

    this->new_slave_port("input", &this->in);

    this->cores_in.reserve(this->nb_cores);
    this->waiting_reqs.resize(this->nb_cores);

    for (int i=0; i<this->nb_cores; i++)
    {
        this->cores_in.emplace_back(i, &ClusterRegisters::core_req);
        this->new_slave_port("input_" + std::to_string(i), &this->cores_in[i]);
    }

    this->external_irq_itf.resize(this->nb_cores);
    for (int i=0; i<this->nb_cores; i++)
    {
        this->new_master_port("external_irq_" + std::to_string(i), &this->external_irq_itf[i]);
    }

    this->regmap.build(this, &this->trace, "regmap");
    this->regmap.cl_clint_set.register_callback(std::bind(&ClusterRegisters::cl_clint_set_req, this, _1, _2, _3, _4));
    this->regmap.cl_clint_clear.register_callback(std::bind(&ClusterRegisters::cl_clint_clear_req, this, _1, _2, _3, _4));
    this->regmap.hw_barrier.register_callback(std::bind(&ClusterRegisters::hw_barrier_req, this, _1, _2, _3, _4));
}

vp::IoReqStatus ClusterRegisters::core_req(vp::Block *__this, vp::IoReq *req, int id)
{
    ClusterRegisters *_this = (ClusterRegisters *)__this;
    uint64_t offset = req->get_addr();
    bool is_write = req->get_is_write();
    uint64_t size = req->get_size();
    uint8_t *data = req->get_data();

    _this->core_access = id;

    _this->trace.msg("Received IO req (offset: 0x%lx, size: 0x%lx, is_write: %d)\n", offset, size, is_write);

    _this->regmap.access(offset, size, data, is_write);

    // Barrier insert 10 cycle stall even for last one to wake-up, seem the request go through AXI
    req->inc_latency(11);
    req->set_resp_status(vp::IO_RESP_OK);

    if (_this->stall_core)
    {
        _this->waiting_reqs[id] = req;
        _this->stall_core = false;
        return vp::IO_REQ_GRANTED;
    }
    else
    {
        return vp::IO_REQ_DONE;
    }
}

vp::IoReqStatus ClusterRegisters::req(vp::Block *__this, vp::IoReq *req)
{
    ClusterRegisters *_this = (ClusterRegisters *)__this;
    uint64_t offset = req->get_addr();
    bool is_write = req->get_is_write();
    uint64_t size = req->get_size();
    uint8_t *data = req->get_data();

    _this->core_access = -1;

    _this->trace.msg("Received IO req (offset: 0x%lx, size: 0x%lx, is_write: %d)\n", offset, size, is_write);

    _this->regmap.access(offset, size, data, is_write);

    req->set_resp_status(vp::IO_RESP_OK);
    return vp::IO_REQ_DONE;
}

void ClusterRegisters::reset(bool active)
{
    this->new_reg("barrier_status", &this->barrier_status, 0, true);

    if (!active)
    {
        this->waiting_cores = 0;
        this->stall_core = false;
    }
}


void ClusterRegisters::cl_clint_set_req(uint64_t reg_offset, int size, uint8_t *value, bool is_write)
{
    this->regmap.cl_clint_set.update(reg_offset, size, value, is_write);
    for (int i=0; i<this->nb_cores; i++)
    {
        int irq_status = (this->regmap.cl_clint_set.get() >> i) & 1;
        if (irq_status == 1)
        {
            this->external_irq_itf[i].sync(true);
        }
    }
}

void ClusterRegisters::hw_barrier_req(uint64_t reg_offset, int size, uint8_t *value, bool is_write)
{
    if (this->core_access != -1)
    {
        this->barrier_status.set(this->barrier_status.get() | (1 << this->core_access));

        if (this->barrier_status.get() == (1ULL << this->nb_cores) - 1)
        {
            this->trace.msg(vp::Trace::LEVEL_DEBUG, "Barrier reached\n");

            this->barrier_status.set(0);

            for (int i=0; i<this->nb_cores; i++)
            {
                if ((this->waiting_cores >> i) & 1)
                {
                    this->trace.msg(vp::Trace::LEVEL_DEBUG, "Wakeup core waiting on barrier (core: %d)\n",
                        i);
                    // Barrier insert 10 cycle stall even for last one to wake-up, seem the request go through AXI
                    vp::IoReq *waiting_req = this->waiting_reqs[i];
                    this->waiting_reqs[i] = NULL;
                    waiting_req->inc_latency(11);
                    this->cores_in[i].resp(waiting_req);
                }
            }

            this->waiting_cores = 0;
        }
        else
        {
            this->trace.msg(vp::Trace::LEVEL_DEBUG, "Stall core due to barrier not reached (core: %d)\n",
                this->core_access);

            this->waiting_cores |= 1 << this->core_access;
            this->stall_core = true;
            return;
        }
    }

    this->stall_core = false;
    return;
}

void ClusterRegisters::cl_clint_clear_req(uint64_t reg_offset, int size, uint8_t *value, bool is_write)
{
    this->regmap.cl_clint_clear.update(reg_offset, size, value, is_write);
    for (int i=0; i<this->nb_cores; i++)
    {
        int irq_status = (this->regmap.cl_clint_clear.get() >> i) & 1;
        if (irq_status == 1)
        {
            this->external_irq_itf[i].sync(false);
        }
    }
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new ClusterRegisters(config);
}
