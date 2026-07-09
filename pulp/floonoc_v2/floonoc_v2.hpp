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

#include <vector>
#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>

/**
 * Shared definitions of the v2 FlooNoC model.
 *
 * The mesh is assembled by the Python generator (pulp/floonoc_v2/floonoc_v2.py):
 * routers and network interfaces are standalone components bound together with
 * 'floonoc_link' ports (see floonoc_link_v2.hpp). This header only carries what
 * travels or is shared between them.
 */

/**
 * Subclass of v2 vp::IoReq used to carry per-mesh metadata that the v1 model
 * stored in the IoReq arg-stack. v2 has no arg stack, so we attach the data as
 * regular fields on a subclass that the NI allocates and the routers downcast.
 */
class FloonocReqV2 : public vp::IoReq
{
public:
    // Destination position in the mesh
    int dest_x;
    int dest_y;
    // Source NI position in the mesh. On the request path the destination NI
    // uses it to route the response back; on the response path it is the
    // position the response is coming back to.
    int src_x;
    int src_y;
    // True if the request is travelling on the response path (back to the
    // source NI), false on the request path (towards the target).
    bool is_rsp;
    // Pointer back to the external IoReq (from the master) that this internal
    // request belongs to. Only dereferenced by the source NI.
    vp::IoReq *burst;
    // True if the request travels on the wide network, false for narrow.
    bool wide;
    // True if this is the AR/AW (address) phase of a split request, false if it
    // is the data phase.
    bool is_address;
    // Pre-translation address, used for VCD traces in the routers.
    uint64_t initiator_addr;
    // Payload storage for read-response flits. Read burst requests are
    // data-less (io_v2 beat protocol) and the target's response beats are
    // pooled objects recycled as soon as they are consumed, so a response
    // flit must carry its data slice across the mesh by value — like the RTL
    // chimney's R flits. Write flits keep pointing into the master's buffer
    // (valid until the B ack round-trips) and leave this empty.
    std::vector<uint8_t> payload;

    // Point this flit's data at its own payload, filled from `data`.
    void set_payload(uint8_t *data, uint64_t size)
    {
        this->payload.assign(data, data + size);
        this->set_data(this->payload.data());
    }
};


/**
 * Memory-map entry: range -> target position on the mesh.
 */
class EntryV2
{
public:
    uint64_t base;
    uint64_t size;
    int x;
    int y;
    uint64_t remove_offset;
};
