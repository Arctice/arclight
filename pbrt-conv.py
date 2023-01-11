#!/usr/bin/python3
import tokenize, builtins
import tomlkit
import argparse
from collections import ChainMap
import pathlib
import random


def strip_indents(line):
    if not line: return line
    else: return line.strip() + '\n'


def collect_tokens(path):
    tokens = []
    ignored = [
        tokenize.COMMENT, tokenize.ENDMARKER, tokenize.NEWLINE, tokenize.NL,
        tokenize.INDENT, tokenize.DEDENT
    ]
    with tokenize.open(path) as f:
        lines = (strip_indents(l) for l in f.readlines())
        stream = tokenize.generate_tokens(lambda: builtins.next(lines))
        tokens = [(t.type, t.string) for t in stream if t.type not in ignored]
        tokens = [(tokenize.tok_name[t[0]], t[1]) for t in tokens]
    tokens.reverse()
    return tokens


def collect(tokens):
    peek = tokens[-1][1]
    if peek == '[':
        tokens.pop()
        xs = []
        while True:
            _, x = tokens.pop()
            if x == ']':
                break
            elif x == '-':
                xs.append(x + tokens.pop()[1])
            else:
                xs.append(x)
        return xs
    elif peek == '-':
        tokens.pop()
        return [peek + tokens.pop()[1]]
    else:
        return [tokens.pop()[1]]


def parse_string(tokens):
    ss = collect(tokens)
    ss = [s.strip('"') for s in ss]
    if len(ss) == 1:
        return ss[0]
    return ss


def parse_number(tokens):
    xs = collect(tokens)
    xs = [float(x) for x in xs]
    if len(xs) == 1:
        return xs[0]
    return xs


def parse_boolean(tokens):
    ss = parse_string(tokens)
    if ss == "true":
        return True
    elif ss == "false":
        return False
    else:
        raise Exception(f'parse_boolean {ss}')


def next_type(tokens):
    for i in range(len(tokens) - 1, -1, -1):
        t = tokens[i]
        if t[0] in ['STRING', 'NUMBER']:
            return t[0]
    assert (False)


def parse_param(tokens):
    if not tokens[-1][0] == 'STRING':
        return False
    t, name = tokens.pop()[1].strip('"').split()
    if t == 'spectrum' and next_type(tokens) == 'STRING':
        return (name, 'spectrum', parse_string(tokens))
    if t in ('integer', 'float', 'point', 'point2', 'point3', 'normal', 'rgb',
             'color', 'spectrum', 'vector', 'vector3'):
        return (name, parse_number(tokens))
    if t in ('blackbody'):
        return (name, t, parse_number(tokens))
    if t in ('string', 'texture'):
        return (name, parse_string(tokens))
    if t in ('bool'):
        return (name, parse_boolean(tokens))
    raise Exception(f'parse_param unknown {t}, {name}')


def parse_params(tokens):
    type = parse_string(tokens).strip('"')
    params = {'_type': type}
    while True:
        param = parse_param(tokens)
        if not param:
            break
        name, *value = *param,
        while builtins.type(value) == list and len(value) == 1:
            value = value[0]
        params[name] = value
    return params


def parse_next(context, tokens):
    type, x = tokens.pop()
    if x == 'Scale':
        return ('Scale', [
            parse_number(tokens),
            parse_number(tokens),
            parse_number(tokens)
        ])
    if x == 'Rotate':
        return ('Rotate', [
            parse_number(tokens),
            parse_number(tokens),
            parse_number(tokens),
            parse_number(tokens)
        ])
    if x == 'Translate':
        return ('Translate', [
            parse_number(tokens),
            parse_number(tokens),
            parse_number(tokens)
        ])
    if x == 'LookAt':
        params = []
        for _ in range(9):
            params.append(parse_number(tokens))
        return ('LookAt', params)
    if x == 'WorldBegin':
        context.world_begin()
        return
    if x == 'WorldEnd':
        return
    if x == 'ReverseOrientation':
        print(f'ignoring {x}')
        return
    if x == 'CoordSysTransform':
        name = parse_string(tokens)
        print(f'ignoring {x} {name}')
        return
    if x == 'AttributeBegin':
        context.attr_begin()
        return
    if x == 'AttributeEnd':
        context.attr_end()
        return
    if x == 'TransformBegin':
        context.transform_begin()
        return
    if x == 'TransformEnd':
        context.transform_end()
        return
    if x == 'ObjectBegin':
        context.object_begin(parse_string(tokens))
        return
    if x == 'ObjectEnd':
        context.object_end()
        return
    if x == 'ObjectInstance':
        name = parse_string(tokens)
        return ('Instance', name)
    if x == 'Texture':
        name = parse_string(tokens)
        type = parse_string(tokens)  # ignored
        params = parse_params(tokens)
        params['name'] = name
        return ('Texture', params)
    if x in ('Transform', 'ConcatTransform'):
        ts = parse_number(tokens)
        return (x, ts)
    if x == 'Identity':
        return (x, ())
    if x == 'NamedMaterial':
        name = parse_string(tokens).strip('"')
        return ('NamedMaterial', name)
    if x in ('Film', 'Camera', 'Sampler', 'Integrator', 'AreaLightSource',
             'Shape', 'Material', 'MakeNamedMaterial', 'LightSource',
             'MakeNamedMedium'):
        return (x, parse_params(tokens))
    if x == 'MediumInterface':
        a, b = parse_string(tokens), parse_string(tokens),
        print(f'ignoring {x}')
        return
    if x == 'PixelFilter':
        px_filter = parse_string(tokens)
        print(f'ignoring {x}')
        return
    if x == 'Include':
        rpath = parse_string(tokens)
        print(f'include {rpath}')
        path = context.path.parent.joinpath(rpath)
        include = collect_tokens(path)
        tokens.extend(include)
        return
    raise Exception(f'parse_next unknown {type}, {x}')


# http://www.vendian.org/mncharity/dir3/blackbody/
kelvin_table = {
    1000: (1.0000, 0.0337, 0.0000),
    1100: (1.0000, 0.0592, 0.0000),
    1200: (1.0000, 0.0846, 0.0000),
    1300: (1.0000, 0.1096, 0.0000),
    1400: (1.0000, 0.1341, 0.0000),
    1500: (1.0000, 0.1578, 0.0000),
    1600: (1.0000, 0.1806, 0.0000),
    1700: (1.0000, 0.2025, 0.0000),
    1800: (1.0000, 0.2235, 0.0000),
    1900: (1.0000, 0.2434, 0.0000),
    2000: (1.0000, 0.2647, 0.0033),
    2100: (1.0000, 0.2889, 0.0120),
    2200: (1.0000, 0.3126, 0.0219),
    2300: (1.0000, 0.3360, 0.0331),
    2400: (1.0000, 0.3589, 0.0454),
    2500: (1.0000, 0.3814, 0.0588),
    2600: (1.0000, 0.4034, 0.0734),
    2700: (1.0000, 0.4250, 0.0889),
    2800: (1.0000, 0.4461, 0.1054),
    2900: (1.0000, 0.4668, 0.1229),
    3000: (1.0000, 0.4870, 0.1411),
    3100: (1.0000, 0.5067, 0.1602),
    3200: (1.0000, 0.5259, 0.1800),
    3300: (1.0000, 0.5447, 0.2005),
    3400: (1.0000, 0.5630, 0.2216),
    3500: (1.0000, 0.5809, 0.2433),
    3600: (1.0000, 0.5983, 0.2655),
    3700: (1.0000, 0.6153, 0.2881),
    3800: (1.0000, 0.6318, 0.3112),
    3900: (1.0000, 0.6480, 0.3346),
    4000: (1.0000, 0.6636, 0.3583),
    4100: (1.0000, 0.6789, 0.3823),
    4200: (1.0000, 0.6938, 0.4066),
    4300: (1.0000, 0.7083, 0.4310),
    4400: (1.0000, 0.7223, 0.4556),
    4500: (1.0000, 0.7360, 0.4803),
    4600: (1.0000, 0.7494, 0.5051),
    4700: (1.0000, 0.7623, 0.5299),
    4800: (1.0000, 0.7750, 0.5548),
    4900: (1.0000, 0.7872, 0.5797),
    5000: (1.0000, 0.7992, 0.6045),
    5100: (1.0000, 0.8108, 0.6293),
    5200: (1.0000, 0.8221, 0.6541),
    5300: (1.0000, 0.8330, 0.6787),
    5400: (1.0000, 0.8437, 0.7032),
    5500: (1.0000, 0.8541, 0.7277),
    5600: (1.0000, 0.8642, 0.7519),
    5700: (1.0000, 0.8740, 0.7760),
    5800: (1.0000, 0.8836, 0.8000),
    5900: (1.0000, 0.8929, 0.8238),
    6000: (1.0000, 0.9019, 0.8473),
    6100: (1.0000, 0.9107, 0.8707),
    6200: (1.0000, 0.9193, 0.8939),
    6300: (1.0000, 0.9276, 0.9168),
    6400: (1.0000, 0.9357, 0.9396),
    6500: (1.0000, 0.9436, 0.9621),
    6600: (1.0000, 0.9513, 0.9844),
    6700: (0.9937, 0.9526, 1.0000),
    6800: (0.9726, 0.9395, 1.0000),
    6900: (0.9526, 0.9270, 1.0000),
    7000: (0.9337, 0.9150, 1.0000),
    7100: (0.9157, 0.9035, 1.0000),
    7200: (0.8986, 0.8925, 1.0000),
    7300: (0.8823, 0.8819, 1.0000),
    7400: (0.8668, 0.8718, 1.0000),
    7500: (0.8520, 0.8621, 1.0000),
    7600: (0.8379, 0.8527, 1.0000),
    7700: (0.8244, 0.8437, 1.0000),
    7800: (0.8115, 0.8351, 1.0000),
    7900: (0.7992, 0.8268, 1.0000),
    8000: (0.7874, 0.8187, 1.0000),
    8100: (0.7761, 0.8110, 1.0000),
    8200: (0.7652, 0.8035, 1.0000),
    8300: (0.7548, 0.7963, 1.0000),
    8400: (0.7449, 0.7894, 1.0000),
    8500: (0.7353, 0.7827, 1.0000),
    8600: (0.7260, 0.7762, 1.0000),
    8700: (0.7172, 0.7699, 1.0000),
    8800: (0.7086, 0.7638, 1.0000),
    8900: (0.7004, 0.7579, 1.0000),
    9000: (0.6925, 0.7522, 1.0000),
    9100: (0.6848, 0.7467, 1.0000),
    9200: (0.6774, 0.7414, 1.0000),
    9300: (0.6703, 0.7362, 1.0000),
    9400: (0.6635, 0.7311, 1.0000),
    9500: (0.6568, 0.7263, 1.0000),
    9600: (0.6504, 0.7215, 1.0000),
    9700: (0.6442, 0.7169, 1.0000),
    9800: (0.6382, 0.7124, 1.0000),
    9900: (0.6324, 0.7081, 1.0000),
    10000: (0.6268, 0.7039, 1.0000),
    10100: (0.6213, 0.6998, 1.0000),
    10200: (0.6161, 0.6958, 1.0000),
    10300: (0.6109, 0.6919, 1.0000),
    10400: (0.6060, 0.6881, 1.0000),
    10500: (0.6012, 0.6844, 1.0000),
    10600: (0.5965, 0.6808, 1.0000),
    10700: (0.5919, 0.6773, 1.0000),
    10800: (0.5875, 0.6739, 1.0000),
    10900: (0.5833, 0.6706, 1.0000),
    11000: (0.5791, 0.6674, 1.0000),
    11100: (0.5750, 0.6642, 1.0000),
    11200: (0.5711, 0.6611, 1.0000),
    11300: (0.5673, 0.6581, 1.0000),
    11400: (0.5636, 0.6552, 1.0000),
    11500: (0.5599, 0.6523, 1.0000),
    11600: (0.5564, 0.6495, 1.0000),
    11700: (0.5530, 0.6468, 1.0000),
    11800: (0.5496, 0.6441, 1.0000),
    11900: (0.5463, 0.6415, 1.0000),
    12000: (0.5431, 0.6389, 1.0000),
    12100: (0.5400, 0.6364, 1.0000),
    12200: (0.5370, 0.6340, 1.0000),
    12300: (0.5340, 0.6316, 1.0000),
    12400: (0.5312, 0.6293, 1.0000),
    12500: (0.5283, 0.6270, 1.0000),
    12600: (0.5256, 0.6247, 1.0000),
    12700: (0.5229, 0.6225, 1.0000),
    12800: (0.5203, 0.6204, 1.0000),
    12900: (0.5177, 0.6183, 1.0000),
    13000: (0.5152, 0.6162, 1.0000),
    13100: (0.5128, 0.6142, 1.0000),
    13200: (0.5104, 0.6122, 1.0000),
    13300: (0.5080, 0.6103, 1.0000),
    13400: (0.5057, 0.6084, 1.0000),
    13500: (0.5035, 0.6065, 1.0000),
    13600: (0.5013, 0.6047, 1.0000),
    13700: (0.4991, 0.6029, 1.0000),
    13800: (0.4970, 0.6012, 1.0000),
    13900: (0.4950, 0.5994, 1.0000),
    14000: (0.4930, 0.5978, 1.0000)
}


def transform_to_camera(T):
    import numpy as np
    T = np.linalg.inv(np.array(T).reshape(4, 4).transpose())
    origin = T @ [0, 0, 0, 1]
    towards = T @ [0, 0, 1, 1]
    up = T @ [0, 1, 0, 0]
    return list(origin), list(towards), list(up)


def trianglemesh_to_ply(t):
    indices = t.get('indices', [0, 1, 2, 1, 2, 3])
    indices = [int(i) for i in indices]
    vertices = t['P']
    assert (0 == len(indices) % 3)
    assert (0 == len(vertices) % 3)
    normals = t.get('N')
    texcoords = t.get('uv') or t.get('st')
    ply = 'ply\nformat ascii 1.0\n'
    ply += f'element vertex {len(vertices) // 3}\n'
    ply += 'property float x\nproperty float y\nproperty float z\n'
    if normals:
        ply += 'property float nx\nproperty float ny\nproperty float nz\n'
    if texcoords:
        ply += 'property float u\nproperty float v\n'
    ply += f'element face {len(indices) // 3}\n'
    ply += 'property list uint uint vertex_indices\n'
    ply += 'end_header\n'
    for idx in range(0, len(vertices) // 3):
        vx = vertices[idx * 3:idx * 3 + 3]
        ply += f'{vx[0]} {vx[1]} {vx[2]}'
        if normals:
            ns = normals[idx * 3:idx * 3 + 3]
            ply += f' {ns[0]} {ns[1]} {ns[2]}'
        if texcoords:
            uv = texcoords[idx * 2:idx * 2 + 2]
            ply += f' {uv[0]} {uv[1]}'
        ply += '\n'
    for idx in range(0, len(indices), 3):
        idxs = indices[idx:idx + 3]
        ply += f'3 {idxs[0]} {idxs[1]} {idxs[2]}\n'
    return ply


class Context:
    def __init__(self, path):
        self.path = pathlib.Path(path)
        self.data_path = self.path.parent.joinpath('arcl-data')
        self.data_path.mkdir(exist_ok=True)
        self.state = ChainMap()
        self.film = {}
        self.camera = {}
        self.models = {}
        self.textures = {}
        self.materials = {}
        self.nodes = {}
        self.renames = {}
        self.lights = None
        self.world = None
        self.instance = None

    def world_begin(self):
        self.world = []
        self.lights = []
        self.state = ChainMap()

    def attr_begin(self):
        self.state = self.state.new_child()

    def attr_end(self):
        self.state = self.state.parents

    def object_begin(self, name):
        self.attr_begin()
        self.state['object_name'] = name
        self.instance = []

    def object_end(self):
        name = self.state['object_name']
        objs = self.instance
        self.nodes[name] = objs
        self.attr_end()
        self.instance = None

    def transform_begin(self):
        # wrong (but will do for some scenes)
        # should keep track of these separately
        self.state = self.state.new_child()

    def transform_end(self):
        self.state = self.state.parents

    def add_node(self, node):
        if self.instance is not None:
            self.instance.append(node)
        elif self.world is not None:
            self.world.append(node)
        else:
            raise Exception("bad node push")

    def push_node(self, node):
        material = None
        if 'material' in self.state:
            material = self.state.get('area-light', False)
            material = material or self.state['material']
            if 'alpha' in self.state:
                A = {
                    'type': 'alpha',
                    'map': self.state['alpha'],
                    'base': material
                }
                alpha = '_malpha' + str(len(self.materials))
                self.materials[alpha] = A
                material = alpha
            node['material'] = material
        if 'transform' in self.state:
            T = node.get('transform', [])
            node['transform'] = T + self.state['transform']
        return self.add_node(node)

    def dump_node(self, node):
        if type(node) is list:
            if len(node) == 1:
                return self.dump_node(node[0])
            g = tomlkit.table()
            g.add('group', True)
            n = 1
            for subnode in node:
                g.add(str(n), self.dump_node(subnode))
                n += 1
            return g

        t = tomlkit.table()
        if 'triangle' in node:
            t.add('triangle', node['triangle'])
            if 'uv' in node:
                t.add('uv', node['uv'])
        elif 'instance' in node:
            ref = '.'.join(node['instance'])
            t.add('instance', ref)
        if 'transform' in node:
            t.add('transform', node['transform'])
        if 'material' in node:
            t.add('material', node['material'])
        return t

    def dump(self):
        scene = tomlkit.document()
        film = tomlkit.table()
        film.add('resolution', self.film.get('resolution', [600, 600]))
        film.add('supersampling', self.film.get('supersampling', 16))
        film.add('depth', self.film.get('depth', 5))
        film.add('global_radiance', self.film.get('global_radiance', 0))
        scene.add("film", film)

        camera = tomlkit.table()
        if 'position' in self.camera:
            for k in 'type', 'position', 'towards', 'up', 'scale':
                camera.add(k, self.camera[k])
        scene.add('camera', camera)

        lights = tomlkit.aot()
        for node in self.lights:
            l = tomlkit.table()
            for k, v in node.items():
                l.add(k, v)
            lights.append(l)
        scene.add('light', lights)

        models = tomlkit.table()
        for name, path in self.models.items():
            m = tomlkit.table()
            m.add('file', path)
            models.add(name, m)
        scene.add('model', models)

        textures = tomlkit.table()
        for name, texture in self.textures.items():
            tx = tomlkit.table()
            for k, v in texture.items():
                tx.add(k, v)
            textures.add(name, tx)
        scene.add('texture', textures)

        materials = tomlkit.table()
        for name, material in self.materials.items():
            mat = tomlkit.table()
            for k, v in material.items():
                mat.add(k, v)
            materials.add(name, mat)
        scene.add('material', materials)

        nodes = tomlkit.table()
        for name, objs in self.nodes.items():
            n = self.dump_node(objs)
            nodes.add(name, n)
        scene.add("node", nodes)

        world = tomlkit.aot()
        for node in self.world:
            n = self.dump_node(node)
            world.append(n)
        scene.add('world', world)

        return tomlkit.dumps(scene)

    def convert_material(self, m):

        if m['type'] == 'matte':
            material = {
                'type': 'lambertian',
                'reflectance': m.get('Kd', [0.5, 0.5, 0.5])
            }

        elif m['type'] == 'area-light-diffuse':
            L = m.get('L', [1, 1, 1])
            scale = m.get('scale', [1, 1, 1])
            material = {'type': 'emissive', 'light': L, 'scale': scale}

        elif m['type'] == 'translucent':
            material = {
                'type': 'translucent',
                'reflectance': m.get('Kd', [0.8, 0.8, 0.8]),
                'transmission': m.get('transmit', [0.4, 0.4, 0.4])
            }

        elif m['type'] in ('uber', 'substrate', 'plastic', 'kdsubsurface'):
            roughness = m.get('uroughness', False) or m.get('roughness', 0.1)
            eta = m.get('eta', m.get('index', [1.5, 1.5, 1.5]))

            coat_type = 'glossy'
            if type(roughness) is float and m.get('remaproughness', True):
                roughness **= 0.5
            if type(roughness) is float and roughness < 0.001:
                coat_type = 'specular'
            D = {
                'type': 'lambertian',
                'reflectance': m.get('Kd', [0.5, 0.5, 0.5]),
            }
            S = {
                'type': coat_type,
                'transmission': 1,
                'roughness': roughness,
                'refraction': eta,
                'absorption': [0, 0, 0],
            }
            N = str(len(self.materials))
            diffuse = '_mix_diffuse' + N
            self.materials[diffuse] = self.material_fixup(D)
            glossy = '_mix_glossy' + N
            self.materials[glossy] = self.material_fixup(S)
            specularity = 1
            if m['type'] == 'plastic':
                specularity = m.get('Ks', 1)
            material = {
                'type': 'coat',
                'weight': specularity,
                'coat': glossy,
                'base': diffuse
            }

        elif m['type'] == 'metal':
            gloss_type = 'glossy'
            roughness = m.get('uroughness', False) or m.get('roughness', 0.01)
            if type(roughness) is float and m.get('remaproughness', True):
                roughness **= 0.5
            if type(roughness) is float and roughness < 0.001:
                gloss_type = 'specular'
            material = {
                'type': gloss_type,
                'transmission': 0,
                'roughness': roughness,
                'refraction': m.get('eta', ['spectrum', 'metal-Cu-eta']),
                'absorption': m.get('k', ['spectrum', 'metal-Cu-k']),
            }

        elif m['type'] == 'glass':
            roughness = m.get('uroughness', False) or m.get('roughness', False)
            eta = m.get('eta', m.get('index', [1.5, 1.5, 1.5]))

            if roughness:
                if type(roughness) is float and m.get('remaproughness', True):
                    roughness **= 0.5
                material = {
                    'type': 'glossy',
                    'transmission': m.get('Kt', [1, 1, 1]),
                    'refraction': eta,
                    'absorption': [0, 0, 0],
                    'roughness': roughness,
                }
            else:
                material = {
                    'type': 'specular',
                    'transmission': m.get('Kt', [1, 1, 1]),
                    'refraction': eta,
                    'absorption': [0, 0, 0]
                }

        elif m['type'] == 'mirror':
            material = {
                'type': 'specular',
                'transmission': [0, 0, 0],
                'refraction': m.get('eta', [100, 100, 100]),
                'absorption': [0.1, 0.1, 0.1]
            }

        elif m['type'] == 'mix':
            if 'materials' in m:
                a, b = m['materials']
            else:
                a, b = m['namedmaterial1'], m['namedmaterial2']
            material = {
                'type': 'coat',
                'weight': m.get('amount', 0.5),
                'coat': a,
                'base': b,
            }

        elif m['type'] in ['fourier', 'measured']:
            # make up something shiny
            print(f'ignored {m["type"]} bsdf {m}')
            D = {
                'type': 'lambertian',
                'reflectance': m.get('Kd', [0.6, 0.6, 0.6]),
            }
            S = {
                'type': 'glossy',
                'refraction': [1.5, 1.5, 1.5],
                'absorption': [0, 0, 0],
                'transmission': 1,
                'roughness': 0.1,
            }
            N = str(len(self.materials))
            diffuse = '_f_diffuse' + N
            self.materials[diffuse] = self.material_fixup(D)
            glossy = '_f_glossy' + N
            self.materials[glossy] = self.material_fixup(S)
            material = {
                'type': 'coat',
                'weight': 1,
                'coat': glossy,
                'base': diffuse
            }

        # pbrt-v4
        elif m['type'] == 'conductor':
            gloss_type = 'glossy'
            roughness = m.get('uroughness', False) or m.get('roughness', 0)
            if type(roughness) is float and m.get('remaproughness', True):
                roughness **= 0.5
            if type(roughness) is float and roughness < 0.001:
                gloss_type = 'specular'
            if 'reflectance' in m:
                print('ignoring reflectance in conductor')
            material = {
                'type': gloss_type,
                'transmission': 0,
                'roughness': roughness,
                'refraction': m.get('eta', ['spectrum', 'metal-Cu-eta']),
                'absorption': m.get('k', ['spectrum', 'metal-Cu-k']),
            }

        elif m['type'] == 'dielectric':
            gloss_type = 'glossy'
            roughness = m.get('uroughness', False) or m.get('roughness', 0)
            if type(roughness) is float and m.get('remaproughness', True):
                roughness **= 0.5
            if type(roughness) is float and roughness < 0.001:
                gloss_type = 'specular'
            material = {
                'type': gloss_type,
                'transmission': 1,
                'roughness': roughness,
                'refraction': m.get('eta', [1.5, 1.5, 1.5]),
                'absorption': m.get('k', [0, 0, 0])
            }

        elif m['type'] == 'diffuse':
            material = {
                'type': 'lambertian',
                'reflectance': m.get('reflectance', [0.5, 0.5, 0.5])
            }

        elif m['type'] == 'diffusetransmission':
            material = {
                'type': 'translucent',
                'reflectance': m.get('reflectance', [0.25, 0.25, 0.25]),
                'transmission': m.get('transmit', [0.25, 0.25, 0.25])
            }

        elif m['type'] == 'coateddiffuse':
            # albedo = m.get('albedo', 0)  # ignored
            # g = m.get('g', 0)  # ignored
            roughness = m.get('roughness', 0)
            coat_type = 'glossy'
            if type(roughness) is float and m.get('remaproughness', True):
                roughness **= 0.5
            if type(roughness) is float and roughness < 0.001:
                coat_type = 'specular'
            D = {
                'type': 'lambertian',
                'reflectance': m.get('reflectance', [0.5, 0.5, 0.5]),
            }
            eta = [1.5, 1.5, 1.5]
            S = {
                'type': coat_type,
                'transmission': 1,
                'roughness': roughness,
                'refraction': eta,
                'absorption': [0, 0, 0],
            }
            N = str(len(self.materials))
            diffuse = '_mix_diffuse' + N
            self.materials[diffuse] = self.material_fixup(D)
            glossy = '_mix_glossy' + N
            self.materials[glossy] = self.material_fixup(S)
            specularity = 1
            material = {
                'type': 'coat',
                'weight': specularity,
                'coat': glossy,
                'base': diffuse
            }

        elif m['type'] == 'coatedconductor':
            # albedo = m.get('albedo', 0)  # ignored
            # g = m.get('g', 0)  # ignored

            i_roughness = m.get('interface.roughness', 0)
            c_roughness = m.get('conductor.roughness', 0)
            coat_type = 'glossy'
            base_type = 'glossy'
            if type(i_roughness) is float and m.get('interface.remaproughness',
                                                    True):
                i_roughness **= 0.5
            if type(c_roughness) is float and m.get('conductor.remaproughness',
                                                    True):
                c_roughness **= 0.5
            if type(i_roughness) is float and i_roughness < 0.001:
                coat_type = 'specular'
            if type(c_roughness) is float and c_roughness < 0.001:
                base_type = 'specular'

            if 'reflectance' in m:
                print('ignoring reflectance in coatedconductor')
            C = {
                'type': 'glossy',
                'refraction': m.get('conductor.eta',
                                    ['spectrum', 'metal-Cu-eta']),
                'absorption': m.get('conductor.k', ['spectrum', 'metal-Cu-k']),
                'transmission': 0,
                'roughness': c_roughness,
            }

            eta = [1.5, 1.5, 1.5]
            S = {
                'type': coat_type,
                'transmission': 1,
                'roughness': i_roughness,
                'refraction': eta,
                'absorption': [0, 0, 0],
            }

            N = str(len(self.materials))
            conductor = '_mix_conductor' + N
            self.materials[conductor] = self.material_fixup(C)
            glossy = '_mix_glossy' + N
            self.materials[glossy] = self.material_fixup(S)
            specularity = 1
            material = {
                'type': 'coat',
                'weight': specularity,
                'coat': glossy,
                'base': conductor
            }

        else:
            raise Exception(f'material type {m["type"]}')

        return self.material_fixup(material)

    def add(self, x):
        t, v = x
        if t == 'Scale':
            T_prev = self.state.get('transform', [])
            T_next = [["scale", v]] + T_prev
            self.state['transform'] = T_next
        elif t == 'Translate':
            T_prev = self.state.get('transform', [])
            T_next = [["translate", v]] + T_prev
            self.state['transform'] = T_next
        elif t == 'Transform':
            T_prev = self.state.get('transform', [])
            T_next = [["matrix", v]]
            self.state['transform'] = T_next
        elif t == 'Identity':
            T_prev = self.state.get('transform', [])
            T_next = []
            self.state['transform'] = T_next
        elif t == 'ConcatTransform':
            T_prev = self.state.get('transform', [])
            T_next = [["matrix", v]] + T_prev
            self.state['transform'] = T_next
        elif t == 'LookAt':
            self.camera['position'] = v[:3]
            self.camera['towards'] = v[3:6]
            self.camera['up'] = v[6:]
        elif t == 'Film':
            xres = v.get('xresolution', 600)
            yres = v.get('yresolution', 600)
            self.film['resolution'] = (xres, yres)
        elif t == 'Camera':
            if not 'position' in self.camera:
                T = (self.state.get('transform', []))
                assert (len(T) == 1 and T[0][0] == 'matrix')
                position, towards, up = transform_to_camera(T[0][1])
                self.camera['position'] = position
                self.camera['towards'] = towards
                self.camera['up'] = up
            type_ = v['_type']
            self.camera['type'] = type_
            default_scale = 60 if (type_ == 'perspective') else 8
            self.camera['scale'] = v.get('fov', default_scale)
        elif t == 'Sampler':
            self.film['supersampling'] = v.get('pixelsamples', 16)
        elif t == 'Integrator':
            self.film['depth'] = v.get('maxdepth', 5)

        elif t == 'Rotate':
            angle, x, y, z = v
            print(f'ignored rotate {v}')

        elif t == 'MakeNamedMaterial':
            name = self.new_material_name(v['_type'])
            material = self.convert_material(v)
            self.materials[name] = material

        elif t == 'MakeNamedMedium':
            print(f'ignored MakeNamedMedium {v}')

        elif t == 'Texture':
            name = self.new_texture_name(v['name'])
            if v['_type'] == 'imagemap':
                path = v['filename']
                self.textures[name] = {'type': 'image', 'file': path}
            elif v['_type'] == 'constant':
                self.textures[name] = {
                    'type': 'constant',
                    'color': v.get('value', [1, 1, 1])
                }
            elif v['_type'] == 'scale':
                self.textures[name] = {
                    'type': 'product',
                    'A': v.get('tex1') or v.get('tex', [1, 1, 1]),
                    'B': v.get('tex2') or v.get('scale', [1, 1, 1]),
                }
            elif v['_type'] == 'mix':
                print(f'used product texture instead of lerp for {v}')
                self.textures[name] = {
                    'type': 'product',
                    'A': v.get('tex1', [1, 1, 1]),
                    'B': v.get('tex2', [1, 1, 1])
                }
            elif v['_type'] == 'marble':
                print(f'ignored marble texture {v}')
                self.textures[name] = {'type': 'constant', 'color': 0.5}
            elif v['_type'] == 'checkerboard':
                self.textures[name] = {
                    'type': 'checkerboard',
                }
            elif v['_type'] in ['fbm', 'wrinkled', 'windy']:
                print(f'ignored noise texture {v}')
                self.textures[name] = {'type': 'constant', 'color': 0.5}
            else:
                raise Exception(f'texture type {v["_type"]}')

        elif t == 'NamedMaterial':
            self.state['material'] = v
        elif t == 'Material':
            if builtins.type(v) is str:
                self.state['material'] = v
            else:
                name = '_m' + str(len(self.materials))
                v['type'] = v['_type']
                material = self.convert_material(v)
                self.materials[name] = material
                self.state['material'] = name

        elif t == 'LightSource':
            L = v.get('L', [1, 1, 1])
            scale = v.get('scale', [1, 1, 1])
            if type(scale) is float:
                scale = [scale] * 3
            if v['_type'] == 'infinite':
                self.film['global_radiance'] = [
                    l * s for l, s in zip(L, scale)
                ]
            elif v['_type'] in ['distant', 'spot']:
                from_ = v.get('from', [0, 0, 0])
                to_ = v.get('to', [0, 0, 1])
                light = {
                    'type': 'distant',
                    'from': from_,
                    'to': to_,
                    'value': L,
                    'scale': scale,
                }
                light = self.material_fixup(light)
                self.lights.append(light)
            else:
                # self.film['global_radiance'] = [1, 1, 1]
                print(f'ignored {v}')

        elif t == 'AreaLightSource':
            name = '_L' + str(len(self.materials))
            v['type'] = 'area-light-diffuse'  #v['_type']
            material = self.convert_material(v)
            self.materials[name] = material
            self.state['material'] = name
            self.state['area-light'] = name

        elif (t == 'Shape'
              and v['_type'] in ('trianglemesh', 'bilinearmesh', 'loopsubdiv')
              and self.world is not None):
            ref = f'mesh{len(self.models)+1}'
            ref = self.new_unique_name(ref)
            ply_path = f'{ref}.ply'
            self.models[ref] = 'arcl-data/' + ply_path
            ply_path = self.data_path.joinpath(ply_path)
            ply = trianglemesh_to_ply(v)
            with open(ply_path, 'w') as out:
                out.write(ply)
            mesh = {'instance': ('model', ref)}
            if 'alpha' in v:
                self.state['alpha'] = v['alpha']
            self.push_node(mesh)
            if 'alpha' in v:
                self.state.pop('alpha')

        elif (t == 'Shape' and v['_type'] in ['sphere', 'disk']
              and self.world is not None):
            path = '../sphere.ply'
            ref = f'm{len(self.models)+1}'
            self.models[ref] = path
            node = {'instance': ('model', ref)}
            if 'radius' in v:
                node['transform'] = [['scale', v['radius']]]
            self.push_node(node)

        elif (t == 'Shape' and v['_type'] == 'plymesh'
              and self.world is not None):
            path = v['filename']
            ref = f'm{len(self.models)+1}'
            self.models[ref] = path
            mesh = {'instance': ('model', ref)}
            if 'alpha' in v:
                self.state['alpha'] = v['alpha']
            self.push_node(mesh)
            if 'alpha' in v:
                self.state.pop('alpha')

        elif (t == 'Instance'):
            inst = {'instance': ('node', v)}
            self.push_node(inst)

        else:
            raise Exception(f'add: {x}')

    def new_texture_name(self, name):
        reused = name in self.textures
        if not reused: return name
        new = self.renames.get(name, name + "-") + f'{random.randint(0, 9)}'
        print(f'renaming {name} to {new}')
        self.renames[name] = new
        return new

    def new_material_name(self, name):
        reused = name in self.materials
        if not reused: return name
        new = self.renames.get(name, name + "-") + f'{random.randint(0, 9)}'
        print(f'renaming {name} to {new}')
        self.renames[name] = new
        return new

    def new_unique_name(self, name):
        reused = False
        if name in self.textures: reused = True
        if name in self.models: reused = True
        if name in self.materials: reused = True
        if name in self.nodes: reused = True
        if not reused:
            return name
        new = self.renames.get(name, name + "-") + f'{random.randint(0, 9)}'
        print(f'renaming {name} to {new}')
        self.renames[name] = new
        return new

    def material_fixup(self, material):
        for k, v in material.items():
            if type(v) == list and v[0] == 'spectrum':
                material[k] = self.convert_spectrum(v[1])
            if type(v) == list and v[0] == 'blackbody':
                material[k] = self.convert_blackbody(v[1])
            if type(v) == str and v in self.renames:
                material[k] = self.renames[v]
        if material['type'] == 'emissive' and 'scale' in material:
            scale = material['scale']
            if type(scale) is float:
                scale = [scale] * 3
            L = material['light']
            L = [l * s for l, s in zip(L, scale)]
            material['light'] = L
        return material

    def convert_spectrum(self, name):
        builtin = {
            'metal-Ag-eta': [0.0569, 0.054, 0.0468],
            'metal-Ag-k': [4.25, 3.4290, 2.8028],
            'metal-Au-eta': [0.16, 0.35, 1.5],
            'metal-Au-k': [4, 2.5, 1.9],
            'metal-Al-eta': [1.345, 0.965, 0.61],
            'metal-Al-k': [7.474, 6.4, 5.3],
            'metal-Cu-eta': [0.271, 0.677, 1.316],
            'metal-Cu-k': [3.61, 2.62, 2.29],
            'metal-CuZn-eta': [0.445, 0.5682, 0.947],
            'metal-CuZn-k': [3.522, 2.588, 1.92],
            'metal-TiO2-eta': [2.585, 2.667, 2.776],
            'metal-TiO2-k': [0, 0, 0],
        }

        if name in builtin:
            return builtin[name]

        path = self.path.parent.joinpath(name)
        values = [l.strip().split() for l in open(path, 'r').readlines()]
        values = [l for l in values if '#' not in l]
        values = [(float(x[0]), float(x[1])) for x in values]
        blue = sorted(values, key=lambda x: abs(x[0] - 465))[0][1]
        green = sorted(values, key=lambda x: abs(x[0] - 532))[0][1]
        red = sorted(values, key=lambda x: abs(x[0] - 630))[0][1]
        return [red, green, blue]

    def convert_blackbody(self, value):
        L = value
        scale = 1
        if L is list:
            if len(L) == 2:
                L, scale = L
            else:
                L = L[0]
        r, g, b = kelvin_table[int(L / 100) * 100]
        return [r * scale, g * scale, b * scale]


args = argparse.ArgumentParser()
args.add_argument('in_file')
args.add_argument('out_file', nargs='?')
args = args.parse_args()
path = args.in_file
out_path = args.out_file

if out_path is None:
    out_path = path.removesuffix('.pbrt') + '.toml'

tokens = collect_tokens(path)
scene = []
context = Context(path)
while tokens:
    p = tokens[-1]
    next = parse_next(context, tokens)
    if next is not None:
        scene.append(next)
        context.add(next)

print(f'done parsing, saving to {out_path}')
toml = context.dump()
with open(out_path, 'w') as out:
    out.write(toml)
