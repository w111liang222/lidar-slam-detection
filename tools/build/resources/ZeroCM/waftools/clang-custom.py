#!/usr/bin/env python
import re
from waflib.Configure import conf

def configure(ctx):
    ctx.find_program('clang',   var='CLANG',   mandatory=True)
    ctx.find_program('clang++', var='CLANG++', mandatory=True)

@conf
def assert_clang_version(ctx, minVer):
    reg = re.compile(r'.*clang version ([0-9]*.[0-9]*).*',re.M)
    out = ctx.cmd_and_log(ctx.env['CLANG']+['--version'])
    ver_s = reg.findall(out)[0].split('.')
    ver_i = tuple([int(s) for s in ver_s[0:2]])

    msg = 'Checking for Clang version'
    res = '.'.join(map(str,ver_i))

    if (float(res) < minVer):
        ctx.msg(msg, res, color='RED')
        ctx.fatal('clang version is too old')
    else:
        ctx.msg(msg, res, color='GREEN')

    return ver_i

@conf
def set_clang_compiler(ctx):
    # This is because we link cpp against c. Sanitizers require clang++ for
    # linking in that case
    ctx.env['LINK_CC'] = ctx.env['CLANG++']
    ctx.env['COMPILER_CC'] = ctx.env['CC'] = ctx.env['CLANG']
    ctx.env['LINK_CXX'] = ctx.env['COMPILER_CXX'] = ctx.env['CXX'] = ctx.env['CLANG++']
