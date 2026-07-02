#
# Copyright (C) 2020 GreenWaves Technologies, SAS, ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from enum import IntEnum
import gvsoc.systree
from gvsoc.signature import IoV2Beat


class FlooNocV2Direction(IntEnum):
    RIGHT = -4
    LEFT = -3
    UP = -2
    DOWN = -1


# Direction indices of the router link ports. Must match RouterV2::DIR_* in
# floonoc_router_v2.hpp.
DIR_RIGHT = 0
DIR_LEFT = 1
DIR_UP = 2
DIR_DOWN = 3
DIR_LOCAL = 4

_DIR_NAMES = ['right', 'left', 'up', 'down', 'local']
_DIR_OPPOSITE = {DIR_RIGHT: DIR_LEFT, DIR_LEFT: DIR_RIGHT, DIR_UP: DIR_DOWN, DIR_DOWN: DIR_UP}

# Network indices of the NI link ports. Must match NetworkInterfaceV2::NW_* in
# floonoc_network_interface_v2.hpp.
NW_REQ = 0
NW_RSP = 1
NW_WIDE = 2

_NW_NAMES = ['req', 'rsp', 'wide']
_NW_ROUTER_PREFIXES = ['req_router_', 'rsp_router_', 'wide_router_']


class FloonocRouterV2(gvsoc.systree.Component):
    """One FlooNoC mesh router (one physical network at one tile).

    Five 'floonoc_link' inputs and five outputs, indexed by direction. Ports of
    absent neighbours are simply left unbound.
    """

    def __init__(self, parent: gvsoc.systree.Component, name, x: int, y: int,
            dim_x: int, dim_y: int, queue_size: int):
        super().__init__(parent, name)

        self.add_sources(['pulp/floonoc_v2/floonoc_router_v2.cpp'])

        self.add_property('x', x)
        self.add_property('y', y)
        self.add_property('dim_x', dim_x)
        self.add_property('dim_y', dim_y)
        self.add_property('router_input_queue_size', queue_size)

    def i_INPUT(self, dir: int) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, f'input_{_DIR_NAMES[dir]}', signature='floonoc_link')

    def o_OUTPUT(self, dir: int, itf: gvsoc.systree.SlaveItf):
        self.itf_bind(f'output_{_DIR_NAMES[dir]}', itf, signature='floonoc_link')


class FloonocNetworkInterfaceV2(gvsoc.systree.Component):
    """One FlooNoC network interface (mesh entry/exit point at one position).

    External ports speak the v2 io protocol; three 'floonoc_link' output ports
    inject into the routers of the req/rsp/wide networks and three input ports
    receive what they deliver.
    """

    def __init__(self, parent: gvsoc.systree.Component, name, x: int, y: int,
            narrow_width: int, wide_width: int, ni_outstanding_reqs: int):
        super().__init__(parent, name)

        self.add_sources(['pulp/floonoc_v2/floonoc_network_interface_v2.cpp'])

        self.narrow_width = narrow_width
        self.wide_width = wide_width

        self.add_property('x', x)
        self.add_property('y', y)
        self.add_property('narrow_width', narrow_width)
        self.add_property('wide_width', wide_width)
        self.add_property('ni_outstanding_reqs', ni_outstanding_reqs)

    def i_NARROW_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'narrow_input', signature=IoV2Beat(self.narrow_width))

    def i_WIDE_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'wide_input', signature=IoV2Beat(self.wide_width))

    def o_NARROW_OUTPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('narrow_output', itf, signature=IoV2Beat(self.narrow_width))

    def o_WIDE_OUTPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('wide_output', itf, signature=IoV2Beat(self.wide_width))

    def i_LINK(self, nw: int) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, f'{_NW_NAMES[nw]}_link_in', signature='floonoc_link')

    def o_LINK(self, nw: int, itf: gvsoc.systree.SlaveItf):
        self.itf_bind(f'{_NW_NAMES[nw]}_link_out', itf, signature='floonoc_link')


class FlooNocV22dMeshNarrowWide(gvsoc.systree.Component):
    """FlooNoC v2 (io v2 protocol) instance for a 2D mesh.

    Mirrors the v1 FlooNoc2dMeshNarrowWide generator API, but is a pure Python
    composite: add_router() and add_network_interface() instantiate real
    router/NI components, and finalize() binds the mesh together with
    'floonoc_link' ports. External ports speak the v2 io protocol
    (vp/itf/io_v2.hpp) — burst beats with is_first/is_last/burst_id and the
    retry() deny handshake.
    """
    def __init__(self, parent: gvsoc.systree.Component, name, narrow_width: int, wide_width:int,
            dim_x: int, dim_y:int, ni_outstanding_reqs: int=8, router_input_queue_size: int=2):
        super().__init__(parent, name)

        self.add_property('mappings', {})
        self.add_property('routers', [])
        self.add_property('network_interfaces', [])
        self.add_property('ni_outstanding_reqs', ni_outstanding_reqs)
        self.add_property('narrow_width', narrow_width)
        self.add_property('wide_width', wide_width)
        self.add_property('dim_x', dim_x)
        self.add_property('dim_y', dim_y)
        self.add_property('router_input_queue_size', router_input_queue_size)

        # (x, y) -> [req, rsp, wide] router components / NI component. Children
        # are created eagerly in add_router()/add_network_interface() so that
        # callers can keep configuring the noc after construction (mappings,
        # extra routers); only the link bindings are deferred to finalize().
        self._routers = {}
        self._nis = {}

    def __add_mapping(self, name: str, base: int, size: int, x: int, y: int, remove_offset:int =0):
        self.get_property('mappings')[name] =  {'base': base, 'size': size, 'x': x, 'y': y, 'remove_offset':remove_offset}

    def add_router(self, x: int, y: int):
        self.get_property('routers').append([x, y])

        self._routers[(x, y)] = [
            FloonocRouterV2(self, f'{_NW_ROUTER_PREFIXES[nw]}{x}_{y}', x=x, y=y,
                dim_x=self.get_property('dim_x'), dim_y=self.get_property('dim_y'),
                queue_size=self.get_property('router_input_queue_size'))
            for nw in range(len(_NW_NAMES))
        ]

    def add_network_interface(self, x: int, y: int):
        self.get_property('network_interfaces').append([x, y])

        narrow_width = self.get_property('narrow_width')
        wide_width = self.get_property('wide_width')

        ni = FloonocNetworkInterfaceV2(self, f'ni_{x}_{y}', x=x, y=y,
            narrow_width=narrow_width, wide_width=wide_width,
            ni_outstanding_reqs=self.get_property('ni_outstanding_reqs'))
        self._nis[(x, y)] = ni

        # External port pass-throughs. These must be registered eagerly (not in
        # finalize) so the composite-level virtual ports are part of the
        # generated ports list.
        self.itf_bind(f'narrow_input_{x}_{y}', ni.i_NARROW_INPUT(),
            signature=IoV2Beat(narrow_width), composite_bind=True)
        self.itf_bind(f'wide_input_{x}_{y}', ni.i_WIDE_INPUT(),
            signature=IoV2Beat(wide_width), composite_bind=True)
        ni.o_NARROW_OUTPUT(gvsoc.systree.SlaveItf(self, f'ni_narrow_{x}_{y}',
            signature=IoV2Beat(narrow_width)))
        ni.o_WIDE_OUTPUT(gvsoc.systree.SlaveItf(self, f'ni_wide_{x}_{y}',
            signature=IoV2Beat(wide_width)))

    def o_NARROW_MAP(self, itf: gvsoc.systree.SlaveItf, base: int, size: int,
            x: int, y: int, name: str=None, rm_base: bool=False, remove_offset:int =0):
        if name is None:
            name = itf.component.name
        if rm_base and remove_offset == 0:
            remove_offset = base
        self.__add_mapping(f"narrow_{name}", base=base, size=size, x=x, y=y, remove_offset=remove_offset)
        self.itf_bind(f"ni_narrow_{x}_{y}", itf, signature=IoV2Beat(self.get_property('narrow_width')))

    def o_WIDE_BIND(self, itf: gvsoc.systree.SlaveItf, x: int, y: int):
        self.itf_bind(f"ni_wide_{x}_{y}", itf, signature=IoV2Beat(self.get_property('wide_width')))

    def o_NARROW_BIND(self, itf: gvsoc.systree.SlaveItf, x: int, y: int):
        self.itf_bind(f"ni_narrow_{x}_{y}", itf, signature=IoV2Beat(self.get_property('narrow_width')))

    def o_MAP_DIR(self, base: int, size: int, dir: FlooNocV2Direction, name: str,
            rm_base: bool=False, remove_offset:int =0):
        if rm_base and remove_offset == 0:
            remove_offset = base
        self.__add_mapping(f"ni_{name}", base=base, size=size, x=dir, y=0, remove_offset=remove_offset)

    def o_MAP(self, base: int, size: int,
            x: int, y: int,
            rm_base: bool=False, remove_offset:int =0):
        if rm_base and remove_offset == 0:
            remove_offset = base
        self.__add_mapping(f"ni_{x}_{y}", base=base, size=size, x=x, y=y, remove_offset=remove_offset)

    def i_NARROW_INPUT(self, x: int, y: int) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, f'narrow_input_{x}_{y}',
            signature=IoV2Beat(self.get_property('narrow_width')))

    def i_WIDE_INPUT(self, x: int, y: int) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, f'wide_input_{x}_{y}',
            signature=IoV2Beat(self.get_property('wide_width')))

    def _ni_router_pos(self, x: int, y: int):
        """Position of the router an NI attaches to.

        NIs on tiles without a router (mesh borders) attach to the nearest
        adjacent router: x+1 first, then x-1, y+1, y-1.
        """
        dim_x = self.get_property('dim_x')
        dim_y = self.get_property('dim_y')

        r_x, r_y = x, y
        if (r_x, r_y) not in self._routers:
            r_x = x + 1

            if x == dim_x - 1 or (r_x, r_y) not in self._routers:
                r_x = x - 1

                if x == 0 or (r_x, r_y) not in self._routers:
                    r_x = x
                    r_y = y + 1

                    if y >= dim_y - 1 or (r_x, r_y) not in self._routers:
                        r_y = y - 1
                        if y == 0:
                            r_x = x
                            r_y = y

        if (r_x, r_y) not in self._routers:
            raise RuntimeError(f'No router found for network interface (position: ({x}, {y}))')

        return r_x, r_y

    @staticmethod
    def _dir_from(router_x: int, router_y: int, from_x: int, from_y: int) -> int:
        """Direction of a router port serving position (from_x, from_y)."""
        if from_x != router_x:
            return DIR_LEFT if from_x < router_x else DIR_RIGHT
        elif from_y != router_y:
            return DIR_DOWN if from_y < router_y else DIR_UP
        else:
            return DIR_LOCAL

    def finalize(self):
        # Every NI holds the full memory map so it can translate addresses to
        # mesh positions on its own. Distributed here so callers can keep
        # adding mappings after construction.
        mappings = self.get_property('mappings')
        for ni in self._nis.values():
            ni.add_property('mappings', mappings)

        # Router meshes: bind each router's directional outputs to the
        # matching input of the neighbouring router of the same network.
        # Directions with no router are left for the NI attachment below
        # (border NIs) or stay unbound (mesh edges).
        for (x, y), routers in self._routers.items():
            neighbours = {DIR_RIGHT: (x+1, y), DIR_LEFT: (x-1, y), DIR_UP: (x, y+1), DIR_DOWN: (x, y-1)}
            for dir, pos in neighbours.items():
                neighbour = self._routers.get(pos)
                if neighbour is not None:
                    for nw in range(len(_NW_NAMES)):
                        routers[nw].o_OUTPUT(dir, neighbour[nw].i_INPUT(_DIR_OPPOSITE[dir]))

        # NI attachment: each NI injects into and receives from the routers of
        # the three networks at its attachment position, through the router
        # ports facing the NI's own position (LOCAL for same-tile NIs, the
        # facing directional ports for border NIs).
        for (x, y), ni in self._nis.items():
            r_x, r_y = self._ni_router_pos(x, y)
            dir = self._dir_from(r_x, r_y, x, y)
            for nw in range(len(_NW_NAMES)):
                router = self._routers[(r_x, r_y)][nw]
                ni.o_LINK(nw, router.i_INPUT(dir))
                router.o_OUTPUT(dir, ni.i_LINK(nw))


class FlooNocV2ClusterGridNarrowWide(FlooNocV22dMeshNarrowWide):
    """FlooNoC v2 instance for a grid of clusters (mirrors v1's variant)."""
    def __init__(self, parent: gvsoc.systree.Component, name, wide_width: int, narrow_width:int, nb_x_clusters: int,
            nb_y_clusters, router_input_queue_size=2, ni_outstanding_reqs: int=2):
        super().__init__(parent, name, wide_width=wide_width, narrow_width=narrow_width,
            dim_x=nb_x_clusters+2, dim_y=nb_y_clusters+2,
            router_input_queue_size=router_input_queue_size,
            ni_outstanding_reqs=ni_outstanding_reqs)

        for tile_x in range(0, nb_x_clusters):
            for tile_y in range(0, nb_y_clusters):
                self.add_router(tile_x+1, tile_y+1)
        for tile_x in range(0, nb_x_clusters+2):
            for tile_y in range(0, nb_y_clusters+2):
                if not ((tile_x == 0 and tile_y == 0) or (tile_x == 0 and tile_y == nb_y_clusters+1) \
                        or (tile_x == nb_x_clusters+1 and tile_y == 0) or \
                        (tile_x == nb_x_clusters+1 and tile_y == nb_y_clusters+1)):
                    self.add_network_interface(tile_x, tile_y)

    def i_CLUSTER_NARROW_INPUT(self, x: int, y: int) -> gvsoc.systree.SlaveItf:
        return self.i_NARROW_INPUT(x+1, y+1)

    def i_CLUSTER_WIDE_INPUT(self, x: int, y: int) -> gvsoc.systree.SlaveItf:
        return self.i_WIDE_INPUT(x+1, y+1)
