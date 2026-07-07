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

#include <string>
#include <climits>
#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include "floonoc_v2.hpp"
#include "floonoc_network_interface_v2.hpp"

NetworkQueueV2::NetworkQueueV2(NetworkInterfaceV2 &ni, std::string name, uint64_t width, int nw)
: vp::Block(&ni, name), ni(ni), width(width), nw(nw)
{
    this->traces.new_trace("trace", &trace, vp::DEBUG);
}

void NetworkQueueV2::reset(bool active)
{
    if (active)
    {
        while (this->queue.size() > 0)
        {
            this->queue.pop();
        }
        this->stalled = false;
    }
}

void NetworkQueueV2::check()
{
    if (!this->stalled && this->queue.size() > 0)
    {
        this->send_router_req();
    }
}

void NetworkQueueV2::unstall()
{
    this->trace.msg(vp::Trace::LEVEL_TRACE, "Unstalling queue\n");
    this->stalled = false;
    this->ni.fsm_event.enqueue();
}

void NetworkQueueV2::handle_req(vp::IoReq *req, bool wide)
{
    this->trace.msg(vp::Trace::LEVEL_DEBUG, "Received %s burst from initiator (burst: %p, offset: 0x%x, size: 0x%x, is_write: %d, op: %d)\n",
                     wide ? "wide" : "narrow", req, req->get_addr(), req->get_size(), req->get_is_write(), req->get_opcode());

    // wide here is the EXTERNAL burst's wide flag (which port it came in on),
    // independent of which internal NetworkQueueV2 (req/rsp/wide) carries the
    // router_reqs. The destination NI uses router_req->wide to pick which
    // target (wide_output_itf vs narrow_output_itf) to forward to, so it must
    // match the input port, not the carrier network.
    //
    // enqueue_router_req frames wormhole packets by destination run: the write
    // data (W beats) to one mesh position stay contiguous (matching the RTL
    // chimney, where the id-less AXI W beats must not interleave), while the
    // address header and any entry-boundary-crossing fragments become their own
    // single-flit packets. Reads (AR) are single-flit and never lock.
    this->enqueue_router_req(req, true, wide, true);
    if (req->get_is_write())
    {
        this->enqueue_router_req(req, false, wide, true);
    }
}

void NetworkQueueV2::handle_rsp(FloonocReqV2 *req, bool is_address)
{
    this->enqueue_router_rsp(req, is_address);
}


void NetworkQueueV2::enqueue_router_req(vp::IoReq *req, bool is_address, bool wide, bool is_req)
{
    uint64_t burst_base = req->get_addr();
    uint64_t burst_size = req->get_size();
    uint8_t *burst_data = req->get_data();
    // Previous flit's destination, for dest-run packet framing below. INT_MIN
    // sentinel so the first flit always opens a run.
    int prev_dest_x = INT_MIN, prev_dest_y = INT_MIN;

    while(burst_size > 0)
    {
        // Address phases (read AR and write AW) are metadata-only headers —
        // one FloonocReqV2 per burst. Data and response phases are paced at
        // the carrier queue's beat width. Beat-based downstreams may answer
        // a forwarded read with a multi-beat resp() stream; the NI's
        // wide_response / narrow_response path is built to absorb that
        // (each beat forwards as a partial response chunk; the FloonocReqV2
        // is only released on the last beat).
        uint64_t size = is_address ? burst_size : std::min(this->width, burst_size);
        FloonocReqV2 *router_req = new FloonocReqV2();

        router_req->prepare();
        router_req->src_x = this->ni.x;
        router_req->src_y = this->ni.y;
        router_req->is_rsp = false;
        router_req->burst = req;
        router_req->is_address = is_address;
        router_req->wide = wide;
        router_req->set_size(size);
        router_req->set_data(burst_data);
        router_req->set_addr(burst_base);
        router_req->set_is_write(req->get_is_write());
        router_req->set_opcode(req->get_opcode());
        router_req->set_second_data(req->get_second_data());

        if (wide)
        {
            req->get_is_write() ? this->ni.wide_write_pending_burst_nb_req++ :
                this->ni.wide_read_pending_burst_nb_req++;
        }
        else
        {
            req->get_is_write() ? this->ni.narrow_write_pending_burst_nb_req++ :
                this->ni.narrow_read_pending_burst_nb_req++;
        }

        EntryV2 *entry = this->ni.get_entry(burst_base, size);

        if (entry == NULL)
        {
            this->trace.msg(vp::Trace::LEVEL_ERROR, "No entry found for base 0x%x\n", burst_base);
            return;
        }
        uint64_t entry_end = entry->base + entry->size;
        uint64_t max_size = entry_end - burst_base;
        size = std::min(max_size, size);

        this->trace.msg(vp::Trace::LEVEL_TRACE,
            "Enqueue request to router (req: %p, base: 0x%x, size: 0x%x, "
            "destination: (%d, %d))\n",
            router_req, burst_base, size, entry->x, entry->y);

        router_req->set_size(size);
        router_req->set_addr(burst_base - entry->remove_offset);
        router_req->initiator_addr = burst_base;
        router_req->dest_x = entry->x;
        router_req->dest_y = entry->y;

        // Wormhole packet framing by DESTINATION run: a packet may only span
        // flits going to the same mesh position, because it reserves a router
        // output along one route until its tail flit. A burst that crosses a
        // memory-map entry boundary (e.g. a 256KB transfer spilling across
        // 64KB per-node regions) fragments into flits with different dests —
        // each same-dest run is its own packet. Framing this way (rather than
        // one packet per burst) is what stops the head flit of one run from
        // leaking a lock on an output the tail flit — bound elsewhere — never
        // releases. is_first opens a run when the dest changes; is_last closes
        // it when the burst ends or the next flit would cross into a new entry.
        bool dest_changed = (entry->x != prev_dest_x || entry->y != prev_dest_y);
        bool reached_entry_end = (burst_base + size >= entry_end);
        bool burst_ends = (burst_size <= size);
        router_req->is_first = dest_changed;
        router_req->is_last = burst_ends || reached_entry_end;
        prev_dest_x = entry->x;
        prev_dest_y = entry->y;

        this->queue.push(router_req);

        burst_base += size;
        burst_data += size;
        burst_size -= size;
    }
    this->ni.fsm_event.enqueue();
}

void NetworkQueueV2::enqueue_router_rsp(FloonocReqV2 *req, bool is_address)
{
    // Allocate a fresh FloonocReqV2 for the response traversal, copying the
    // metadata the router needs from the incoming request. The caller deletes
    // the original after we return — mirrors the v1 enqueue_router_req
    // response-path branch.
    FloonocReqV2 *router_req = new FloonocReqV2();

    router_req->prepare();
    // Response flits are single-flit packets, mirroring the RTL chimney: R
    // beats set hdr.last=1 on every beat ("no reason to do wormhole routing for
    // R bursts" — floo_axi_chimney) and the B ack is a lone flit. Read data
    // therefore never holds a router output across the holes the (slow) target
    // leaves between beats; only writes (AW+W) are wormhole packets.
    router_req->is_first = true;
    router_req->is_last  = true;
    router_req->src_x = req->src_x;
    router_req->src_y = req->src_y;
    router_req->is_rsp = true;
    router_req->burst = req->burst;
    router_req->is_address = is_address;
    router_req->wide = req->wide;
    router_req->dest_x = req->dest_x;
    router_req->dest_y = req->dest_y;
    router_req->initiator_addr = req->initiator_addr;
    router_req->set_size(req->get_size());
    router_req->set_data(req->get_data());
    router_req->set_addr(req->get_addr());
    router_req->set_is_write(req->get_is_write());
    router_req->set_opcode(req->get_opcode());
    router_req->set_second_data(req->get_second_data());

    this->queue.push(router_req);
    this->ni.fsm_event.enqueue();
}

void NetworkQueueV2::send_router_req()
{
    FloonocReqV2 *req = this->queue.front();
    this->queue.pop();
    vp::IoReq *burst = req->burst;

    if (!req->is_rsp)
    {
        int *nb_req;
        if (req->wide)
        {
            nb_req = burst->get_is_write() ? &this->ni.wide_write_pending_burst_nb_req :
                &this->ni.wide_read_pending_burst_nb_req;
        }
        else
        {
            nb_req = burst->get_is_write() ? &this->ni.narrow_write_pending_burst_nb_req :
                &this->ni.narrow_read_pending_burst_nb_req;
        }
        *nb_req = *nb_req - 1;
        if (*nb_req == 0)
        {
            this->ni.fsm_event.enqueue();
        }
    }

    this->trace.msg(vp::Trace::LEVEL_DEBUG, "Handling addr burst (burst: %p, offset: 0x%x, size: 0x%x, is_write: %d, op: %d)\n",
                    burst, burst->get_addr(), burst->get_size(), burst->get_is_write(), burst->get_opcode());

    this->stalled = this->ni.link_out[this->nw].req(req);
    if (this->stalled)
    {
        this->trace.msg(vp::Trace::LEVEL_TRACE, "Stalling network interface (position: (%d, %d))\n", this->ni.x, this->ni.y);
    }

    if (this->queue.size() > 0)
    {
        this->ni.fsm_event.enqueue();
    }
}

NetworkInterfaceV2::NetworkInterfaceV2(vp::ComponentConf &config)
    : vp::Component(config),
      wide_output_itf(&NetworkInterfaceV2::wide_retry, &NetworkInterfaceV2::wide_response),
      narrow_output_itf(&NetworkInterfaceV2::narrow_retry, &NetworkInterfaceV2::narrow_response),
      wide_input_itf(&NetworkInterfaceV2::wide_req),
      narrow_input_itf(&NetworkInterfaceV2::narrow_req),
      link_out{{
        FloonocLinkMaster(NW_REQ, &NetworkInterfaceV2::link_unstall),
        FloonocLinkMaster(NW_RSP, &NetworkInterfaceV2::link_unstall),
        FloonocLinkMaster(NW_WIDE, &NetworkInterfaceV2::link_unstall)
      }},
      link_in{{
        FloonocLinkSlave(NW_REQ, &NetworkInterfaceV2::link_req),
        FloonocLinkSlave(NW_RSP, &NetworkInterfaceV2::link_req),
        FloonocLinkSlave(NW_WIDE, &NetworkInterfaceV2::link_req)
      }},
      fsm_event(this, &NetworkInterfaceV2::fsm_handler),
      signal_narrow_req(*this, "narrow_req", 64),
      signal_wide_req(*this, "wide_req", 64),
      req_queue(*this, "narrow", get_js_config()->get_uint("narrow_width"), NW_REQ),
      rsp_queue(*this, "rsp", get_js_config()->get_uint("narrow_width"), NW_RSP),
      wide_queue(*this, "wide", get_js_config()->get_uint("wide_width"), NW_WIDE),
      response_queue(this, "response_queue", &this->fsm_event)
{
    traces.new_trace("trace", &trace, vp::DEBUG);

    this->x = get_js_config()->get_int("x");
    this->y = get_js_config()->get_int("y");
    this->narrow_width = get_js_config()->get_uint("narrow_width");
    this->wide_width = get_js_config()->get_uint("wide_width");
    this->ni_outstanding_reqs = get_js_config()->get_int("ni_outstanding_reqs");
    this->max_burst_size = get_js_config()->get_uint("max_burst_size");

    this->new_master_port("wide_output", &this->wide_output_itf);
    this->new_master_port("narrow_output", &this->narrow_output_itf);
    this->new_slave_port("narrow_input", &this->narrow_input_itf);
    this->new_slave_port("wide_input", &this->wide_input_itf);

    static const char *nw_names[NW_NB] = {"req", "rsp", "wide"};
    for (int i = 0; i < NW_NB; i++)
    {
        this->new_master_port(std::string(nw_names[i]) + "_link_out", &this->link_out[i]);
        this->new_slave_port(std::string(nw_names[i]) + "_link_in", &this->link_in[i]);
    }

    // Every NI receives the full memory map so it can translate addresses to
    // mesh positions on its own.
    js::Config *mappings = get_js_config()->get("mappings");
    if (mappings != NULL)
    {
        this->entries.reserve(mappings->get_childs().size());
        for (auto& mapping: mappings->get_childs())
        {
            js::Config *config = mapping.second;

            uint64_t base = config->get_uint("base");
            uint64_t size = config->get_uint("size");
            uint64_t remove_offset = config->get_uint("remove_offset");
            int map_x = config->get_int("x");
            int map_y = config->get_int("y");

            EntryV2 entry;
            entry.base = base;
            entry.size = size;
            entry.x = map_x;
            entry.y = map_y;
            entry.remove_offset = remove_offset;
            this->entries.push_back(entry);
        }
    }

    // AXI-style burst legality: a burst may not cross a max_burst_size boundary
    // (the 4KB rule), so it always targets a single mesh position and every
    // wormhole packet is single-destination. For that to hold, no two targets
    // may share a max_burst_size-aligned page. We enforce it the simple way:
    // require every target to be aligned to and a multiple of max_burst_size, so
    // each occupies whole pages and can never share one with another. That is
    // how real AXI slave regions are laid out anyway. Config error, so always
    // active (not just in asserts builds).
    if (this->max_burst_size > 0)
    {
        for (EntryV2 &e : this->entries)
        {
            if (e.size == 0)
            {
                continue;
            }
            if (e.base % this->max_burst_size != 0 || e.size % this->max_burst_size != 0)
            {
                this->trace.fatal("Target (%d,%d) [0x%lx..0x%lx] is not aligned to and a "
                    "multiple of max_burst_size=0x%lx; targets must not share a page so a "
                    "burst always lands in one target\n", e.x, e.y, e.base,
                    e.base + e.size - 1, this->max_burst_size);
            }
        }
    }
}

EntryV2 *NetworkInterfaceV2::get_entry(uint64_t base, uint64_t size)
{
    for (int i=0; i<this->entries.size(); i++)
    {
        EntryV2 *entry = &this->entries[i];
        if (entry->size > 0 && base >= entry->base && base < entry->base + entry->size)
        {
            return entry;
        }
    }
    return NULL;
}

vp::IoRespAck NetworkInterfaceV2::wide_response(vp::Block *__this, vp::IoReq *req)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;
    _this->handle_response(_this->unwrap_response(req));
    return vp::IO_RESP_ACCEPTED;
}

// Recover our own FloonocReqV2 from a downstream response. A beat target answers
// with DISTINCT per-beat objects (initiator-owned convention) carrying our request
// as req->initiator; we fold this beat's per-beat payload (addr/size/data/framing/
// status) onto our own request — reused per beat, exactly as the legacy same-object
// beat-stream model expects — and free the beat. A write ack round-trips our own
// object (req == initiator), so there is nothing to fold or free.
FloonocReqV2 *NetworkInterfaceV2::unwrap_response(vp::IoReq *req)
{
    FloonocReqV2 *self_req = (FloonocReqV2 *)req->initiator;
    if (req != self_req)
    {
        self_req->set_addr(req->get_addr());
        self_req->set_size(req->get_size());
        self_req->set_data(req->get_data());
        self_req->is_first = req->is_first;
        self_req->is_last  = req->is_last;
        self_req->set_resp_status(req->get_resp_status());
        delete req;
    }
    return self_req;
}

void NetworkInterfaceV2::wide_retry(vp::Block *__this, vp::IoRetryChannel)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;
    // The downstream target is ready again. Re-send the req we were holding
    // and unstall the upstream router so further reqs can flow.
    if (_this->wide_target_stalled_req)
    {
        FloonocReqV2 *req = _this->wide_target_stalled_req;
        _this->wide_target_stalled_req = NULL;

        req->prepare();
        vp::IoReqStatus result = _this->wide_output_itf.req(req);
        if (result == vp::IO_REQ_DENIED)
        {
            // Target denied again — hold and wait for next retry.
            _this->wide_target_stalled_req = req;
            return;
        }
        else if (result == vp::IO_REQ_DONE)
        {
            if (req->get_latency() > 0)
            {
                _this->response_queue.push_delayed(req, req->get_latency());
            }
            else
            {
                _this->handle_response(req);
            }
        }
        // GRANTED: async response will arrive via wide_response.

        if (_this->wide_stalled_link_nw != -1)
        {
            _this->link_in[_this->wide_stalled_link_nw].unstall();
            _this->wide_stalled_link_nw = -1;
        }
    }
    _this->fsm_event.enqueue();
}

vp::IoRespAck NetworkInterfaceV2::narrow_response(vp::Block *__this, vp::IoReq *req)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;
    _this->handle_response(_this->unwrap_response(req));
    return vp::IO_RESP_ACCEPTED;
}

void NetworkInterfaceV2::narrow_retry(vp::Block *__this, vp::IoRetryChannel)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;
    if (_this->narrow_target_stalled_req)
    {
        FloonocReqV2 *req = _this->narrow_target_stalled_req;
        _this->narrow_target_stalled_req = NULL;

        req->prepare();
        vp::IoReqStatus result = _this->narrow_output_itf.req(req);
        if (result == vp::IO_REQ_DENIED)
        {
            _this->narrow_target_stalled_req = req;
            return;
        }
        else if (result == vp::IO_REQ_DONE)
        {
            if (req->get_latency() > 0)
            {
                _this->response_queue.push_delayed(req, req->get_latency());
            }
            else
            {
                _this->handle_response(req);
            }
        }

        if (_this->narrow_stalled_link_nw != -1)
        {
            _this->link_in[_this->narrow_stalled_link_nw].unstall();
            _this->narrow_stalled_link_nw = -1;
        }
    }
    _this->fsm_event.enqueue();
}

void NetworkInterfaceV2::reset(bool active)
{
    this->trace.msg(vp::Trace::LEVEL_TRACE, "Resetting network interface\n");
    if (active)
    {
        this->wide_read_pending_burst = NULL;
        this->wide_write_pending_burst = NULL;
        this->wide_read_pending_burst_nb_req = 0;
        this->wide_write_pending_burst_nb_req = 0;
        this->narrow_read_pending_burst = NULL;
        this->narrow_write_pending_burst = NULL;
        this->narrow_read_pending_burst_nb_req = 0;
        this->narrow_write_pending_burst_nb_req = 0;
        this->nb_pending_bursts[0] = 0;
        this->nb_pending_bursts[1] = 0;
        this->owes_retry_wide_input = false;
        this->owes_retry_narrow_input = false;
        this->wide_target_stalled_req = NULL;
        this->narrow_target_stalled_req = NULL;
        this->wide_stalled_link_nw = -1;
        this->narrow_stalled_link_nw = -1;
    }
}

int NetworkInterfaceV2::get_req_nw(bool is_wide, bool is_write)
{
    if (is_wide)
    {
        return is_write ? NetworkInterfaceV2::NW_WIDE : NetworkInterfaceV2::NW_REQ;
    }
    else
    {
        return NetworkInterfaceV2::NW_REQ;
    }
}

int NetworkInterfaceV2::get_rsp_nw(bool is_wide, bool is_write)
{
    return is_wide ? NetworkInterfaceV2::NW_WIDE : NetworkInterfaceV2::NW_RSP;
}

vp::IoReqStatus NetworkInterfaceV2::narrow_req(vp::Block *__this, vp::IoReq *req)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;
    _this->signal_narrow_req = req->get_addr();
    return _this->handle_req(req, /*wide=*/false);
}

vp::IoReqStatus NetworkInterfaceV2::wide_req(vp::Block *__this, vp::IoReq *req)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;
    _this->signal_wide_req = req->get_addr();
    return _this->handle_req(req, /*wide=*/true);
}

vp::IoReqStatus NetworkInterfaceV2::handle_req(vp::IoReq *req, bool wide)
{
    this->trace.msg(vp::Trace::LEVEL_DEBUG, "Received request from target (req: %p, base: 0x%x, size: 0x%x, wide: %d)\n",
        req, req->get_addr(), req->get_size(), wide);

    // AXI burst legality (asserts builds only): a burst must fit in max_burst_size
    // and must not cross a max_burst_size boundary, so it lands in a single page
    // and hence a single target — keeping every wormhole packet single-dest.
    if (this->max_burst_size > 0)
    {
        uint64_t base = req->get_addr(), size = req->get_size();
        this->traces.assert(size <= this->max_burst_size,
            "Input burst size 0x%lx exceeds max_burst_size 0x%lx (addr 0x%lx)",
            size, this->max_burst_size, base);
        this->traces.assert(size == 0 || (base % this->max_burst_size) + size <= this->max_burst_size,
            "Input burst [0x%lx..0x%lx] crosses a max_burst_size=0x%lx boundary (AXI 4KB rule)",
            base, base + size - 1, this->max_burst_size);
    }

    // Use the v2 IoReq remaining_size field to track how many bytes still need
    // to be returned through the mesh before we can ack the burst.
    req->remaining_size = req->get_size();
    // We do NOT clobber req->initiator here: v2 masters such as the iDMA
    // backend use that field for their own bookkeeping (BurstInfo*) and
    // expect it to survive the round-trip through the mesh. The NI instead
    // picks the response port from the inner FloonocReqV2's `wide` flag.

    vp::IoReq **queue;
    if (wide)
    {
        queue = req->get_is_write() ? &this->wide_write_pending_burst :
                &this->wide_read_pending_burst;
    }
    else
    {
        queue = req->get_is_write() ? &this->narrow_write_pending_burst :
                &this->narrow_read_pending_burst;
    }

    // Admission is bounded only by the number of outstanding bursts, matching
    // the RTL chimney's MaxTxns semantics. We deliberately do NOT serialize on
    // the previous burst finishing its fragmentation into router flits: that
    // coupling made admission stall whenever the downstream router was
    // back-pressured (a hotspot), starving the sources closest to the jam in
    // periodic bubbles the RTL does not have. The per-burst pointer below is
    // kept only as an aggregate drain-wakeup helper (see fsm_handler).
    if (this->nb_pending_bursts[wide] >= this->ni_outstanding_reqs)
    {
        // v2 deny: do not queue. Remember that the master is owed a retry()
        // when capacity returns.
        if (wide)
        {
            this->owes_retry_wide_input = true;
        }
        else
        {
            this->owes_retry_narrow_input = true;
        }
        return vp::IO_REQ_DENIED;
    }
    else
    {
        this->nb_pending_bursts[wide]++;
        *queue = req;
        if (!req->get_is_write() || !wide)
        {
            this->req_queue.handle_req(req, wide);
        }
        else
        {
            this->wide_queue.handle_req(req, wide);
        }
        this->fsm_event.enqueue();
        return vp::IO_REQ_GRANTED;
    }
}

bool NetworkInterfaceV2::link_req(vp::Block *__this, FloonocReqV2 *req, int nw)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;

    if (req->is_rsp)
    {
        // Response path: a reply has come back from the destination NI to us
        // (the source NI). Account it on the corresponding burst and reply to
        // the master when the burst is complete.
        _this->trace.msg(vp::Trace::LEVEL_DEBUG, "Received response from router (req: %p)\n", req);

        vp::IoReq *burst = req->burst;
        bool wide = req->wide;

        // The inner FloonocReqV2's `wide` flag tells us which external slave
        // port the burst entered through; we use that to dispatch the
        // response rather than burst->initiator (which belongs to the
        // master, e.g. the iDMA's BurstInfo*).
        vp::IoSlave *port = wide ? &_this->wide_input_itf : &_this->narrow_input_itf;

        if (burst->get_is_write())
        {
            _this->trace.msg(vp::Trace::LEVEL_DEBUG, "Received write burst response (burst: %p)\n",
                burst);
            _this->nb_pending_bursts[wide]--;

            burst->set_resp_status(vp::IO_RESP_OK);
            port->resp(burst);
        }
        else
        {
            _this->trace.msg(vp::Trace::LEVEL_TRACE,
                "Reducing remaining size of burst (burst: %p, size: %d, req: %p, size %d)\n",
                burst, (int)burst->remaining_size, req, (int)req->get_size());
            burst->remaining_size -= req->get_size();

            if (burst->remaining_size == 0)
            {
                _this->trace.msg(vp::Trace::LEVEL_DEBUG, "Finished burst (burst: %p)\n", burst);
                _this->nb_pending_bursts[wide]--;
                burst->set_resp_status(vp::IO_RESP_OK);
                port->resp(burst);
            }
        }

        _this->fsm_event.enqueue();

        delete req;
    }
    else
    {
        // Request path: we are the destination NI. Forward to the local target
        // via our external master port.
        _this->trace.msg(vp::Trace::LEVEL_DEBUG,
            "Received request from router (req: %p, base: 0x%x, size: 0x%x, isaddr: (%d), "
            "position: (%d, %d)) origin Ni: (%d, %d)\n",
            req, req->get_addr(), req->get_size(), (int)req->is_address, _this->x,
            _this->y, req->src_x, req->src_y);

        if ((req->get_is_write() && !req->is_address) || !req->get_is_write())
        {
            bool is_stalled = false;

            bool wide = req->wide;
            vp::IoMaster *target = wide ? &_this->wide_output_itf : &_this->narrow_output_itf;
            _this->trace.msg(vp::Trace::LEVEL_DEBUG,
                "Sending request to target (req: %p, base: 0x%x, size: 0x%x)\n",
                req, req->get_addr(), req->get_size());

            req->prepare();
            // The is_first/is_last on this flit are the mesh wormhole packet
            // framing (set by enqueue_router_req/rsp); prepare() preserves them.
            // They must NOT leak into the io_v2 beat protocol used towards the
            // local target: the beat-to-single-req adapter reads is_first/is_last
            // to delimit a submission, and a mid-packet frame here would make it
            // complete the write early — the ack then frees the burst while the
            // source still has flits queued referencing it (SIGSEGV). Present
            // each forwarded flit as a self-contained single-beat submission.
            req->is_first = true;
            req->is_last = true;
            // Initiator-owned request convention: a beat target answers an async
            // read with DISTINCT response beat objects (not this request reused).
            // Point initiator at ourselves so each response beat carries a back-ref
            // to this FloonocReqV2; the response callbacks recover it via
            // req->initiator instead of assuming the response IS this object.
            req->initiator = req;
            vp::IoReqStatus result = target->req(req);

            if (result == vp::IO_REQ_DONE)
            {
                if (req->get_latency() > 0)
                {
                    _this->response_queue.push_delayed(req, req->get_latency());
                }
                else
                {
                    _this->handle_response(req);
                }
            }
            else if (result == vp::IO_REQ_DENIED)
            {
                // v2 master holds the denied req and re-sends it from the
                // target's retry() callback (wide_retry / narrow_retry). The
                // link the req arrived on is stalled by our return value;
                // remember it so the retry can unstall it.
                if (wide)
                {
                    _this->wide_target_stalled_req = req;
                    _this->wide_stalled_link_nw = nw;
                }
                else
                {
                    _this->narrow_target_stalled_req = req;
                    _this->narrow_stalled_link_nw = nw;
                }
                is_stalled = true;
            }
            // GRANTED: async response will arrive via wide_response/narrow_response.

            return is_stalled;
        }
        else
        {
            // Address-only phase of a split write — no actual transfer to do.
            delete req;
        }
    }

    return false;
}

void NetworkInterfaceV2::link_unstall(vp::Block *__this, int nw)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;
    NetworkQueueV2 *queues[NW_NB] = {&_this->req_queue, &_this->rsp_queue, &_this->wide_queue};
    queues[nw]->unstall();
}


void NetworkInterfaceV2::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    NetworkInterfaceV2 *_this = (NetworkInterfaceV2 *)__this;

    _this->req_queue.check();
    _this->rsp_queue.check();
    _this->wide_queue.check();

    if (!_this->response_queue.empty())
    {
        _this->handle_response((FloonocReqV2 *)_this->response_queue.pop());
    }

    if (_this->wide_read_pending_burst && _this->wide_read_pending_burst_nb_req == 0)
    {
        _this->wide_read_pending_burst = NULL;
        _this->fsm_event.enqueue();
    }

    if (_this->wide_write_pending_burst && _this->wide_write_pending_burst_nb_req == 0)
    {
        _this->wide_write_pending_burst = NULL;
        _this->fsm_event.enqueue();
    }

    if (_this->narrow_read_pending_burst && _this->narrow_read_pending_burst_nb_req == 0)
    {
        _this->narrow_read_pending_burst = NULL;
        _this->fsm_event.enqueue();
    }

    if (_this->narrow_write_pending_burst && _this->narrow_write_pending_burst_nb_req == 0)
    {
        _this->narrow_write_pending_burst = NULL;
        _this->fsm_event.enqueue();
    }

    // v2 deny/retry: if a master was previously denied and we now have capacity
    // (an outstanding burst completed), call retry() once. The master will
    // re-issue when it sees the retry callback. Mirrors the admission gate
    // above: retry as soon as nb_pending_bursts drops below the limit, without
    // waiting for any in-flight fragmentation to drain.
    if (_this->owes_retry_wide_input &&
        _this->nb_pending_bursts[1] < _this->ni_outstanding_reqs)
    {
        _this->owes_retry_wide_input = false;
        _this->wide_input_itf.retry();
    }
    if (_this->owes_retry_narrow_input &&
        _this->nb_pending_bursts[0] < _this->ni_outstanding_reqs)
    {
        _this->owes_retry_narrow_input = false;
        _this->narrow_input_itf.retry();
    }

    _this->response_queue.trigger_next();
}


void NetworkInterfaceV2::handle_response(FloonocReqV2 *req)
{
    if (!req->get_is_write())
    {
        // Read response: forward the data back through the rsp/wide network
        // to the originating NI. handle_rsp allocates a *fresh* FloonocReqV2
        // for the return trip and copies the relevant fields, so the
        // incoming req survives even if we still need it for further beats.
        //
        // io_v2 slaves may answer in three forms — sync DONE, async
        // big-packet (single resp with is_first==is_last==true) or beat
        // stream (N resps reusing the same IoReq, with is_first/is_last
        // mutated per beat and cumulative sizes equal to the request size).
        // For each beat we ship a partial response of req->get_size() bytes;
        // the source NI accumulates ``remaining_size`` and replies to the
        // upstream master when it reaches zero. The FloonocReqV2 is only
        // released once the downstream slave signals is_last — until then
        // it must stay alive for the next beat to mutate.
        req->dest_x = req->src_x;
        req->dest_y = req->src_y;
        if (req->wide)
        {
            this->wide_queue.handle_rsp(req, false);
        }
        else
        {
            this->rsp_queue.handle_rsp(req, false);
        }
    }
    else
    {
        // Write response: only emit one ack through the mesh, after all
        // data-phase requests have arrived.
        vp::IoReq *burst = req->burst;

        this->trace.msg(vp::Trace::LEVEL_TRACE,
            "Reducing remaining size of burst (burst: %p, size: %d, req: %p, size %d)\n",
            burst, (int)burst->remaining_size, req, (int)req->get_size());
        burst->remaining_size -= req->get_size();

        if (burst->remaining_size == 0)
        {
            this->trace.msg(vp::Trace::LEVEL_DEBUG, "Finished burst (burst: %p)\n", burst);
            req->dest_x = req->src_x;
            req->dest_y = req->src_y;
            this->rsp_queue.handle_rsp(req, true);
        }
    }
    // Keep the FloonocReqV2 alive across the beats of a multi-beat resp().
    // Only release it on the last beat — sync DONE and async big-packet
    // both deliver is_last=true on their single resp, so they release
    // immediately as before.
    if (req->is_last)
    {
        delete req;
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new NetworkInterfaceV2(config);
}
