#!/usr/bin/python3
import tokenize, builtins
import tomlkit
import argparse
from collections import ChainMap
import pathlib


def collect_tokens(path):
    tokens = []
    ignored = [
        tokenize.COMMENT, tokenize.ENDMARKER, tokenize.NEWLINE, tokenize.NL,
        tokenize.INDENT, tokenize.DEDENT
    ]
    with tokenize.open(path) as f:
        for token in tokenize.generate_tokens(f.readline):
            if token.type in ignored:
                continue
            type_name = token.__repr__().split('(')[2].split(')')[0]
            tokens.append((type_name, token.string))
    return tokens


def parse_string(tokens):
    ss = []
    if tokens[0][1] == '[':
        tokens.pop(0)
        while True:
            _, x = tokens[0]
            if x == ']':
                tokens.pop(0)
                break
            else:
                ss.append(parse_string(tokens))
    else:
        ss.append(tokens.pop(0)[1].strip('"'))
    if len(ss) == 1:
        ss = ss[0]
    return ss


def parse_param(tokens):
    if not tokens[0][0] == 'STRING':
        return False
    # print('param', tokens[0])
    t, name = tokens.pop(0)[1].strip('"').split()
    if t == 'spectrum' and tokens[0][0] == 'STRING':
        return (name, 'spectrum', parse_string(tokens))
    if t in ('integer', 'float', 'point', 'normal', 'rgb', 'color',
             'spectrum'):
        return (name, parse_number(tokens))
    if t in ('blackbody'):
        return (name, t, parse_number(tokens))
    if t in ('string', 'texture'):
        return (name, parse_string(tokens))
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


def parse_number(tokens):
    type, x = tokens.pop(0)
    if type == 'OP':
        if x == '[':
            xs = []
            while True:
                _, x = tokens[0]
                if x == ']':
                    tokens.pop(0)
                    break
                else:
                    xs.append(parse_number(tokens))
            return xs
        return float(x + str(parse_number(tokens)))
    if type == 'NUMBER':
        return float(x)
    raise Exception(f'parse_number unknown {type}, {x}')


def parse_next(context, tokens):
    type, x = tokens.pop(0)
    if x == 'Scale':
        return ('Scale', [
            parse_number(tokens),
            parse_number(tokens),
            parse_number(tokens)
        ])
    if x == 'LookAt':
        params = []
        for _ in range(9):
            params.append(parse_number(tokens))
        return ('Lookat', params)
    if x == 'WorldBegin':
        context.world_begin()
        return
    if x == 'WorldEnd':
        return
    if x == 'AttributeBegin':
        context.attr_begin()
        return
    if x == 'AttributeEnd':
        context.attr_end()
        return
    if x == 'Texture':
        name = parse_string(tokens)
        type = parse_string(tokens)  # ignored
        params = parse_params(tokens)
        params['name'] = name
        return ('Texture', params)
    if x == 'NamedMaterial':
        name = parse_string(tokens).strip('"')
        return ('NamedMaterial', name)
    if x in ('Film', 'Camera', 'Sampler', 'Integrator', 'AreaLightSource',
             'Shape', 'Material', 'MakeNamedMaterial', 'LightSource'):
        return (x, parse_params(tokens))
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


class Context:
    def __init__(self, path):
        self.path = pathlib.Path(path)
        self.state = ChainMap()
        self.film = {}
        self.camera = {}
        self.models = {}
        self.textures = {}
        self.materials = {}
        self.world = None
        self.instance = None

    def world_begin(self):
        self.world = []
        self.state = ChainMap()

    def attr_begin(self):
        self.state = self.state.new_child()

    def attr_end(self):
        self.state = self.state.parents

    def push_node(self, node):
        if 'material' in self.state:
            node['material'] = self.state['material']
        if self.world is not None:
            self.world.append(node)
        else:
            raise Exception()

    def dump(self):
        scene = tomlkit.document()
        film = tomlkit.table()
        film.add('resolution', self.film.get('resolution', [600, 600]))
        film.add('supersampling', self.film.get('supersampling', 16))
        film.add('depth', self.film.get('depth', 5))
        film.add('global_radiance', self.film.get('global_radiance', 1))
        scene.add("film", film)

        camera = tomlkit.table()
        if 'position' in self.camera:
            for k in 'position', 'towards', 'up':
                camera.add(k, self.camera[k])
        camera.add('scale', 4)
        scene.add('camera', camera)

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

        world = tomlkit.aot()
        for node in self.world:
            t = tomlkit.table()
            if 'instance' in node:
                a, b = node['instance']
                ref = a + '.' + b
                t.add('instance', ref)
            if 'transform' in node:
                t.add('transform', node['transform'])
            if 'material' in node:
                m = node['material']
                t.add('material', m)
            if 'triangle' in node:
                t.add('triangle', node['triangle'])
            world.append(t)
        scene.add('world', world)

        return tomlkit.dumps(scene)

    def convert_material(self, m):
        if m['type'] == 'matte':
            material = {
                'type': 'lambertian',
                'reflectance': m.get('Kd', [0.5, 0.5, 0.5])
            }

        elif m['type'] == 'diffuse':
            material = {
                'type': 'emissive',
                'light': m.get('L', [1, 1, 1]),
            }

        elif m['type'] == 'uber' or m['type'] == 'substrate':
            roughness = m.get('uroughness', False) or m.get('roughness', 0.1)
            D = {
                'type': 'lambertian',
                'reflectance': m.get('Kd', [0.5, 0.5, 0.5]),
            }
            S = {
                'type': 'glossy',
                'transmission': 1,
                'roughness': 0.01,
                'refraction': m.get('eta', [1.5, 1.5, 1.5]),
                'absorption': [0, 0, 0],
            }
            N = str(len(self.materials))
            diffuse = '_mix_diffuse' + N
            self.materials[diffuse] = self.material_fixup(D)
            glossy = '_mix_glossy' + N
            self.materials[glossy] = self.material_fixup(S)
            material = {
                'type': 'coat',
                'weight': 1,
                'coat': glossy,
                'base': diffuse
            }

        elif m['type'] == 'metal':
            roughness = m.get('uroughness', False) or m.get('roughness', 0.01)
            if type(roughness) is float:
                roughness **= 0.5
            material = {
                'type': 'glossy',
                'transmission': 0,
                'roughness': roughness,
                'refraction': m.get('eta', [0.159, 0.145, 0.1135]),
                'absorption': m.get('k', [3.929, 3.19, 2.38])
            }

        elif m['type'] == 'glass':
            material = {
                'type': 'specular',
                'transmission': m.get('Kt', [1, 1, 1]),
                'refraction': m.get('eta', [1.5, 1.5, 1.5]),
                'absorption': [0, 0, 0]
            }

        elif m['type'] == 'mirror':
            material = {
                'type': 'specular',
                'transmission': [0, 0, 0],
                'refraction': m.get('eta', [2, 2, 2]),
                'absorption': [0.01, 0.01, 0.01]
            }

        elif m['type'] == 'mix':
            material = {
                'type': 'coat',
                'weight': m.get('amount', 0.5),
                'coat': m['namedmaterial1'],
                'base': m['namedmaterial2'],
            }

        elif m['type'] == 'fourier':
            print(f'ignored {m}')
            material = {'type': 'lambertian', 'reflectance': [0.5, 0.5, 0.5]}

        else:
            raise Exception(f'material type {m["type"]}')

        return self.material_fixup(material)

    def add(self, x):
        t, v = x
        if t == 'Scale':
            self.state['scale'] = x
        elif t == 'Lookat':
            self.camera['position'] = v[:3]
            self.camera['towards'] = v[3:6]
            self.camera['up'] = v[6:]
        elif t == 'Film':
            xres = v.get('xresolution', 600)
            yres = v.get('yresolution', 600)
            self.film['resolution'] = (xres, yres)
        elif t == 'Camera':
            pass
        elif t == 'Sampler':
            self.film['supersampling'] = v.get('pixelsamples', 16)
        elif t == 'Integrator':
            self.film['depth'] = v.get('maxdepth', 5)

        elif t == 'MakeNamedMaterial':
            name = v['_type']
            material = self.convert_material(v)
            self.materials[name] = material

        elif t == 'Texture':
            name = v['name']
            if v['_type'] == 'imagemap':
                path = v['filename']
                self.textures[name] = {'type': 'image', 'file': path}
            elif v['_type'] == 'scale':
                self.textures[name] = {
                    'type': 'product',
                    'A': v.get('tex1', [1, 1, 1]),
                    'B': v.get('tex2', [1, 1, 1])
                }
            else:
                raise Exception(f'texture type {v["_type"]}')

        elif t == 'NamedMaterial':
            self.state['material'] = v
        elif t == 'Material':
            if type(v) == str:
                self.state['material'] = v
            else:
                name = '_m' + str(len(self.materials))
                v['type'] = v['_type']
                material = self.convert_material(v)
                self.materials[name] = material
                self.state['material'] = name

        elif t == 'LightSource':
            if v['_type'] == 'infinite' and 'L' in v:
                self.film['global_radiance'] = v['L']
            else:
                self.film['global_radiance'] = [1, 1, 1]
                print(f'ignored {v}')

        elif t == 'AreaLightSource':
            name = '_L' + str(len(self.materials))
            v['type'] = v['_type']
            material = self.convert_material(v)
            self.materials[name] = material
            self.state['material'] = name

        elif (t == 'Shape' and v['_type'] == 'trianglemesh'
              and self.world is not None):
            points = v['P']
            indices = [int(i) for i in v['indices']]
            vertices = []
            for idx in range(0, len(points), 3):
                vertices.append(points[idx:idx + 3])
            for idx in range(0, len(indices), 3):
                a, b, c = indices[idx:idx + 3]
                A, B, C = vertices[a], vertices[b], vertices[c]
                self.push_node({'triangle': [A, B, C]})

        elif (t == 'Shape' and v['_type'] == 'sphere'
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
            self.push_node({'instance': ('model', ref)})

        else:
            raise Exception(f'add: {x}')

    def material_fixup(self, material):
        for k, v in material.items():
            if type(v) == list and v[0] == 'spectrum':
                material[k] = self.convert_spectrum(v[1])
            if type(v) == list and v[0] == 'blackbody':
                material[k] = self.convert_blackbody(v[1])
        return material

    def convert_spectrum(self, path):
        path = self.path.parent.joinpath(path)
        values = [l.strip().split() for l in open(path, 'r').readlines()]
        values = [(float(x[0]), float(x[1])) for x in values]
        blue = sorted(values, key=lambda x: abs(x[0] - 450))[0][1]
        green = sorted(values, key=lambda x: abs(x[0] - 540))[0][1]
        red = sorted(values, key=lambda x: abs(x[0] - 600))[0][1]
        return [red, green, blue]

    def convert_blackbody(self, value):
        r, g, b = kelvin_table[int(value[0] / 100) * 100]
        L = value[1]
        return [r * L, g * L, b * L]


args = argparse.ArgumentParser()
args.add_argument('in_file')
args.add_argument('out_file')
args = args.parse_args()
path = args.in_file
out_path = args.out_file

tokens = collect_tokens(path)
scene = []
context = Context(path)
while tokens:
    p = tokens[0]
    next = parse_next(context, tokens)
    if next is not None:
        scene.append(next)
        context.add(next)

with open(out_path, 'w') as out:
    out.write(context.dump())
