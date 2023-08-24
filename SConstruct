AddOption(
    '--sanitize',
    action='store_true',
    help='Enable address sanitization',
    default=False)

debug = ARGUMENTS.get('debug', False)
noopt = ARGUMENTS.get('noopt', False)

env = Environment(
    CXX='clang++',
    CXXFLAGS=[
        '-O3' if not noopt else '-O0',
        '-std=c++20',
        '-fcolor-diagnostics',
        '-Wall',
        '-march=native',
        # -fdiagnostics-color=auto
        # '-v'
    ],
    CPPPATH=['#','/usr/local/include'],
    LIBS=[])

if debug:
    env.Append(CXXFLAGS='-g')

if GetOption('sanitize'):
    env.Append(CXXFLAGS =['-fsanitize=address', '-fno-omit-frame-pointer'])
    env.Append(LINKFLAGS=['-fsanitize=address', '-fno-omit-frame-pointer'])

Export("env")
test = SConscript('test/SConscript', variant_dir=f'build/test')

Default(test)
