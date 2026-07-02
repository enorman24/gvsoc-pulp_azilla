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
#include <list>
#include "floonoc_v2.hpp"
#include "floonoc_link_v2.hpp"

class NetworkInterfaceV2;

/**
 * Per-network injection queue of the NI. Splits external bursts into mesh
 * requests and feeds them to the NI's link output for its network (req, rsp
 * or wide).
 */
class NetworkQueueV2 : public vp::Block
{
    friend class NetworkInterfaceV2;
public:
    NetworkQueueV2(NetworkInterfaceV2 &ni, std::string name, uint64_t width, int nw);
    void reset(bool active) override;

    void check();
    void handle_req(vp::IoReq *req, bool wide);
    void handle_rsp(FloonocReqV2 *req, bool is_address);

private:
    void enqueue_router_req(vp::IoReq *req, bool is_address, bool wide, bool is_req);
    void enqueue_router_rsp(FloonocReqV2 *req, bool is_address);
    void send_router_req();
    void unstall();

    NetworkInterfaceV2 &ni;
    uint64_t width;
    // Network this queue injects into (NetworkInterfaceV2::NW_*), i.e. which
    // of the NI's link output ports it drives.
    int nw;
    vp::Trace trace;
    std::queue<FloonocReqV2 *> queue;
    bool stalled;
};

/**
 * v2 FlooNoC network interface.
 *
 * Standalone component instantiated by the generator: entry/exit point of the
 * mesh. External ports speak the v2 io protocol (burst beats with is_first /
 * is_last / burst_id, plus the retry() deny handshake). Mesh traversal uses
 * FloonocReqV2 over 'floonoc_link' ports bound to the local (or, for border
 * NIs, nearest) routers of the three physical networks.
 */
class NetworkInterfaceV2 : public vp::Component
{
    friend class NetworkQueueV2;

public:
    static constexpr int NW_REQ   = 0;
    static constexpr int NW_RSP   = 1;
    static constexpr int NW_WIDE  = 2;
    static constexpr int NW_NB    = 3;

    NetworkInterfaceV2(vp::ComponentConf &config);

    void reset(bool active);

    void handle_response(FloonocReqV2 *req);
    // Recover our own FloonocReqV2 from a downstream response beat (initiator-owned
    // request convention): folds a distinct beat's payload onto our request and
    // frees the beat; passes a round-tripped write ack through unchanged.
    FloonocReqV2 *unwrap_response(vp::IoReq *req);

private:
    // Link input callback: a mesh request (or response) delivered by a router.
    // Returns true when the NI's downstream target denied it (the router must
    // then stall the corresponding output until unstalled).
    static bool link_req(vp::Block *__this, FloonocReqV2 *req, int nw);
    // Link output callback: the router accepts injections again on network nw.
    static void link_unstall(vp::Block *__this, int nw);
    static vp::IoRespAck wide_response(vp::Block *__this, vp::IoReq *req);
    static void wide_retry(vp::Block *__this, vp::IoRetryChannel);
    static vp::IoRespAck narrow_response(vp::Block *__this, vp::IoReq *req);
    static void narrow_retry(vp::Block *__this, vp::IoRetryChannel);
    static vp::IoReqStatus narrow_req(vp::Block *__this, vp::IoReq *req);
    static vp::IoReqStatus wide_req(vp::Block *__this, vp::IoReq *req);
    vp::IoReqStatus handle_req(vp::IoReq *req, bool wide);
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
    int get_req_nw(bool is_wide, bool is_write);
    int get_rsp_nw(bool is_wide, bool is_write);
    EntryV2 *get_entry(uint64_t base, uint64_t size);

    int ni_outstanding_reqs;
    int x;
    int y;
    uint64_t narrow_width;
    uint64_t wide_width;

    // Memory map, address range -> mesh position. Every NI holds the full
    // table (same 'mappings' property on each).
    std::vector<EntryV2> entries;

    vp::IoMaster wide_output_itf;
    vp::IoMaster narrow_output_itf;
    vp::IoSlave wide_input_itf;
    vp::IoSlave narrow_input_itf;

    // Mesh links, indexed by NW_*: outputs inject into the routers, inputs
    // receive what the routers deliver.
    std::array<FloonocLinkMaster, NW_NB> link_out;
    std::array<FloonocLinkSlave, NW_NB> link_in;

    vp::Trace trace;
    NetworkQueueV2 req_queue;
    NetworkQueueV2 wide_queue;
    NetworkQueueV2 rsp_queue;
    vp::ClockEvent fsm_event;

    vp::Signal<uint64_t> signal_narrow_req;
    vp::Signal<uint64_t> signal_wide_req;

    // Pending external bursts. Only one of each (per wide x read/write) can be
    // accepted at a time, matching v1 semantics.
    vp::IoReq *wide_read_pending_burst;
    vp::IoReq *wide_write_pending_burst;
    int wide_read_pending_burst_nb_req;
    int wide_write_pending_burst_nb_req;
    vp::IoReq *narrow_read_pending_burst;
    vp::IoReq *narrow_write_pending_burst;
    int narrow_read_pending_burst_nb_req;
    int narrow_write_pending_burst_nb_req;

    // owes_retry_*_input: true when an incoming req was DENIED and the master
    // is owed a retry() once capacity returns.
    bool owes_retry_wide_input;
    bool owes_retry_narrow_input;

    // When a downstream target returns DENIED, v2 requires the master (this
    // NI) to hold the req and re-send it on the target's retry(). One slot
    // per output port.
    FloonocReqV2 *wide_target_stalled_req;
    FloonocReqV2 *narrow_target_stalled_req;
    // Link input (NW_* index) to unstall when a target retry frees the
    // corresponding output, -1 when none. This is the link the denied request
    // ARRIVED on, which is not implied by its wide flag (a wide read AR
    // travels on the req network).
    int wide_stalled_link_nw;
    int narrow_stalled_link_nw;

    // Synchronous responses are pushed here so they fire after the latency
    // annotation expires.
    vp::Queue response_queue;

    int nb_pending_bursts[2];
};
