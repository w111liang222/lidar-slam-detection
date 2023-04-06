#!/usr/bin/env python
import os
import re
import copy
from waflib import Task, Utils, Context, Logs
from waflib import TaskGen
from waflib.Utils import subprocess
from waflib.TaskGen import extension, feature
from waflib.Configure import conf

def configure(ctx):
    ctx.find_program('cxxtestgen', var='CXXTESTGEN', mandatory=True)

@conf
def setup_cxxtest(ctx):
    ctx.env.CXXTESTFLAGS_suite   = ['--error-printer', '--part']
    ctx.env.CXXTESTFLAGS_runner  = ['--error-printer', '--root']

@conf
def check_cxxtest_version(ctx):
    reg = re.compile(r'This is CxxTest version\s(.*)',re.M)
    out = ctx.cmd_and_log(ctx.env['CXXTESTGEN']+['--version'])
    ver_s = reg.findall(out)[0].split('.')
    ver_i = tuple([int(s) for s in ver_s[0:2]])

    msg='Checking for cxxtest version'
    ctx.msg(msg,'.'.join(map(str,ver_i)))

    return ver_i

@conf
def cxxtest(ctx, **kw):
    # Check if the 'test' directory exists and if there are any tests in it
    if (ctx.path.find_dir('test') is None):
        return

    suites = ctx.path.ant_glob('test/**/*Test.hpp')
    if (len(suites) == 0):
        return

    # generate runner src
    runner = 'test/runner.cpp'
    ctx(rule      = cxxtest_generate_runner,
        target    = runner,
        shell     = False,
        reentrant = False)

    # generate suite src
    ctx(source = suites)

    # compile list of all src
    cpp_src = [None] * (len(suites) + 1)
    for idx, val in enumerate(suites):
        pathparts = val.abspath().split('/')
        testIdx = (pathparts[::-1].index('test') + 1) * -1
        nameparts = '/'.join(pathparts[testIdx:])
        nameparts = nameparts.replace('.hpp', '.cpp')
        cpp_src[idx] = ctx.path.get_bld().find_or_declare(nameparts)

    cpp_src[-1] = ctx.path.get_bld().find_or_declare(runner)

    # compile test program
    if hasattr(kw, 'target'):
        del kw['target']
    if hasattr(kw, 'includes'):
        del kw['includes']
    if hasattr(kw, 'source'):
        del kw['source']

    kw['use'] += ['cxxtest']

    #if ctx.env.MACHINE == 'x64':
    ctx.program(target   ='test/runner',
                includes = '.',
                source   = cpp_src,
                install_path = None,
                **kw)
    #else:
        #if hasattr(kw, 'features'):
            #del kw['features']
#
        #ctx(features = 'cxx arm_pkg',
            #target   = 'test/runner.tar.gz',
            #includes = '.',
            #source   = cpp_src,
            #**kw)

def cxxtest_generate_suite(tsk):
    pathparts = tsk.inputs[0].abspath().split('/')
    testIdx = (pathparts[::-1].index('test') + 1) * -1
    nameparts = '/'.join(pathparts[testIdx:])
    # adding the --include flag with the relative path to the test suite is
    # required for waf to properly generate the dependency tree, otherwise it
    # misses the dependency on the hpp file
    tsk.exec_command('%s %s -o %s %s --include=%s' % (tsk.env['CXXTESTGEN'][0],
                                                     ' '.join(tsk.env['CXXTESTFLAGS_suite']),
                                                     tsk.outputs[0].abspath(),
                                                     tsk.inputs[0].abspath(),
                                                     nameparts))

def cxxtest_generate_runner(tsk):
    tsk.exec_command('%s %s -o %s' % (tsk.env['CXXTESTGEN'][0],
                                      ' '.join(tsk.env['CXXTESTFLAGS_runner']),
                                      tsk.outputs[0].abspath()))

# Create a task generator that can make test suites
TaskGen.declare_chain(
    name      = 'cxxtest-gen',
    rule      = cxxtest_generate_suite,
    shell     = False,
    reentrant = False,
    ext_out   = 'Test.cpp',
    ext_in    = 'Test.hpp')
