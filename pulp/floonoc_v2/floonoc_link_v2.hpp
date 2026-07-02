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

#pragma once

#include <vp/vp.hpp>

class FloonocReqV2;

// FlooNoC mesh link protocol
// --------------------------
//
// Point-to-point link between two mesh nodes (router <-> router or
// NI <-> router), carrying FloonocReqV2 mesh requests. The protocol is the
// port-based equivalent of the former FloonocNodeV2 virtual-call pair:
//
//   - master.req(req) hands one request to the receiver. The receiver always
//     accepts it (there is no deny) and returns true when this acceptance
//     filled it up — the sender must then stop sending on this link until the
//     receiver calls back unstall(). Ownership of the request transfers on the
//     call: no response ever comes back on the link (read data travels as
//     separate mesh requests on the rsp network).
//
//   - slave.unstall() tells the sender the receiver can accept again.
//
// Both calls execute synchronously in the same cycle, exactly like the
// virtual calls they replace, so link timing is defined entirely by the
// receiver (input-queue delay) and unchanged from the monolithic model.
//
// Each port carries a constructor-time id which is transferred to the peer at
// bind time and passed back as argument on every call: the receiver sees the
// id of its own slave port (which input the request arrived on, replacing the
// from_x/from_y coordinate comparison) and the sender sees the id of its own
// master port (which output the unstall refers to). Python signature:
// 'floonoc_link'.

typedef bool (FloonocLinkReqMeth)(vp::Block *, FloonocReqV2 *req, int id);
typedef void (FloonocLinkUnstallMeth)(vp::Block *, int id);

class FloonocLinkSlave;

class FloonocLinkMaster : public vp::MasterPort
{
    friend class FloonocLinkSlave;

public:
    FloonocLinkMaster(int id, FloonocLinkUnstallMeth *unstall_meth)
        : id(id), unstall_meth(unstall_meth) {}

    // Send one mesh request to the bound receiver. Returns true when the
    // receiver accepted it but is now full (stall this output until it calls
    // back unstall()).
    inline bool req(FloonocReqV2 *req)
    {
        return this->slave_req_meth((vp::Block *)this->get_remote_context(), req,
            this->slave_id);
    }

    inline bool is_bound() { return this->remote_port != NULL; }

    inline void bind_to(vp::Port *port, js::Config *config) override;
    void finalize() override {}

private:
    // Output identifier of this port, passed back by the receiver on
    // unstall() so the owner knows which output became free.
    int id;
    FloonocLinkUnstallMeth *unstall_meth;

    // Peer information captured at bind time.
    FloonocLinkReqMeth *slave_req_meth = NULL;
    int slave_id = -1;
};

class FloonocLinkSlave : public vp::SlavePort
{
    friend class FloonocLinkMaster;

public:
    FloonocLinkSlave(int id, FloonocLinkReqMeth *req_meth)
        : id(id), req_meth(req_meth) {}

    // Tell the bound sender that this input can accept requests again after a
    // previous req() returned true.
    inline void unstall()
    {
        this->master_unstall_meth((vp::Block *)this->get_remote_context(),
            this->master_id);
    }

    inline void bind_to(vp::Port *port, js::Config *config) override;
    void finalize() override {}

private:
    // Input identifier of this port, passed to the receiver's req callback so
    // it knows which input the request arrived on.
    int id;
    FloonocLinkReqMeth *req_meth;

    // Peer information captured at bind time.
    FloonocLinkUnstallMeth *master_unstall_meth = NULL;
    int master_id = -1;
};

inline void FloonocLinkMaster::bind_to(vp::Port *_port, js::Config *config)
{
    FloonocLinkSlave *port = (FloonocLinkSlave *)_port;
    vp::Port::bind_to(_port, config);
    this->slave_req_meth = port->req_meth;
    this->slave_id = port->id;
}

inline void FloonocLinkSlave::bind_to(vp::Port *_port, js::Config *config)
{
    FloonocLinkMaster *port = (FloonocLinkMaster *)_port;
    vp::Port::bind_to(_port, config);
    this->master_unstall_meth = port->unstall_meth;
    this->master_id = port->id;
}
