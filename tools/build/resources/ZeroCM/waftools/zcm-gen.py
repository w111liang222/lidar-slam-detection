#!/usr/bin/env python
# encoding: utf-8

import os
import waflib
from waflib import Task
from waflib.Errors import WafError
from waflib.TaskGen import extension
from waflib.Configure import conf

def configure(ctx):
    # Note: This tool will also work if zcm-gen is a tool built by this build if:
    #       configuration:
    #           ctx.env.ZCMGEN is set to the programs path prior to loading the zcm-gen tool
    #       build:
    #           all zcmtype generation happens after the tool is compiled.
    #           That is, use ctx.add_group() while ctx.post_mode = waflib.Build.POST_LAZY
    #           to separate zcm-gen compilation and zcmtype generation.
    #
    # TODO: could improve this to actually use the waf node of the zcm-gen binary as in the
    #       ex. of building a compiler in the waf book (https://waf.io/book/#_advanced_scenarios)
    # TODO: We should do the above; changes to the zcm-gen binary do not force the regeneration
    #       of the zcmtypes. This is generally not a huge problem because
    #       zcm-gen stays pretty static, but it is technically a build error.
    # TODO: The user MUST have uselib variables for zcm and zcmjar defined through their own
    #       configuration. We should add them here or move them into a different waf module

    # find program leaves the program itself in an array (probably for if there are multiple),
    # but we only expect / want one
    if not getattr(ctx.env, 'ZCMGEN', []):
        ctx.find_program('zcm-gen', var='ZCMGEN', mandatory=True)
        ctx.env.ZCMGEN = ctx.env.ZCMGEN[0]

# ******* ZcmGen Task Generator *******
# @conf
# def zcmgen(ctx, **kw):
#
# Context method used to set up proper include paths and simplify user code for zcmtype builds
# keywords:
#   name:         identifier for uselib linking (e.g. c/c++ builds may simply add 'name'
#                 to their 'use' list to get proper include path and library linking)
#                 default = 'zcmtypes' (though it is encouraged to name it something more unique
#                                       to avoid library naming conflicts)
#   source:       list of zcmtype source files to be interpreted
#   build:        False if only zcmtype generation (no compiling or linking) is desired
#                 Defaults to true.
#   lang:         list of languages for which zcmtypes should be generated, options are:
#                 ['c_stlib', 'c_shlib', 'cpp', 'java', 'python', 'nodejs', 'julia'].
#                 Can also be a space separated string.
#   pkgPrefix:    Optional package prefix that will be prepended to the packages of all zcmtypes
#                 default = ''
#   littleEndian: True or false based on desired endianess of output. Should almost always
#                 be false. Don't use this option unless you really know what you're doing
#   javapkg:      name of the java package
#                 default = 'zcmtypes' (though it is encouraged to name it something more unique
#                                       to avoid library naming conflicts)
#   juliapkg:     Julia module prefix that all types will be generated into. This prefix also
#                 prefixes the global pkgPrefix if one is set.
#                 default = 'zcmtypes' (though it is encouraged to name it something more unique
#                                       to avoid library naming conflicts)
#   juliagenpkgs: If True, generate julia module files for all packages. ALL ZCMTYPES MUST BE
#                 INCLUDED IN SOURCE FOR THIS COMMAND! Types are NOT generated themselves
#                 unless the user specifies 'julia' in the `lang` list. This allows users
#                 to split up type generation and apply options like `littleEndian` to only
#                 a subset of their types. However, It is required that the `juliapkg` option
#                 be identical on this command to the commands that generate the individual
#                 types.
#
#   TODO: add support for changing c/c++ include directory. Currently defaults so that the
#         description below works
#
# Using the output zcmtypes:
#   For the following explanations, assume:
#     ${name} = value entered for the "name" keyword,
#     ${dir}  = dir containing the zcmtypes (eg for ~/zcm/examples/types/*.zcm, ${dir} = "types")
#     ${pkg}  = value entered for the "javapkg" keyword
#
#   C:             types will be compiled into a static library and linked into targets
#     wscript:     add '${name}_c' to the list of "use" dependencies
#     c files:     target include directives at "${dir}/type_name_t.h"
#
#   C++:           types are header only and will be included directly in source
#     wscript:     add '${name}_cpp' to the list of "use" dependencies
#     cpp files:   target include directives at "${dir}/type_name_t.hpp"
#
#   Java:          types will be compiled into a .jar, see details below on execution
#     wscript:     add '${name}_java' to the list of "use" dependencies
#     java files:  target import directives at "$pkg/type_name_t"
#
#   Python:        types will be compiled into .py python files and are imported
#                  into final python script.
#     wscript:     add '${name}_python' to the list of "use" dependencies if you
#                  have any build time dependencies (most python dependencies,
#                  however, are runtime so this will often, if not always, go unused).
#     py files:    add directory containing .py files to your sys path and target
#                  import directives at "$pkg/type_name_t"
#
#   NodeJS:        types will be compiled into a zcmtypes.js file which is
#                  'require'd in your nodejs main app.js file and passed into
#                  nodes zcm constructor
#     wscript:     add '${name}_nodejs to the list of "use" dependencies if you
#                  have any build time dependencies (most nodejs dependencies,
#                  however, are runtime so this will often, if not always, go unused).
#
#   Julia:         types will be compiled into .jl files. Simply `import` them
#                  into the final julia script. Types with packages should be imported
#                  by importing their base level package. Types without packages should
#                  be imported by importing the package "_" + "type_name_t"
#     wscript:     add '${name}_julia' to the list of "use" dependencies
#     julia files: Add ${dir} to your LOAD_PATH : "unshift!(LOAD_PATH, ${dir})"
#                  Import packaged type :         "import pkg_name: type_name_t"
#
# Note on running the output java classes:
#   Because of the way that java's CLASSPATH works, even though waf will link the appropriate jar
#   files for the build, their location is not maintained in the output jar. This means that to
#   run a java program compiled in this manner, the final output jar AND the zcmtype jar MUST be
#   in the CLASSPATH. Indeed, this waf build system will even be able to find the primary zcm.jar
#   if it is not in the CLASSPATH and will be able to build without issue, but any attempt to run
#   the output java classes without having zcm.jar in the CLASSPATH will fail. The zcmtype jar
#   will be called ${name}.jar and will be located in the mirrored build path to wherever the
#   waf zcmgen tool was invoked.

# returns filenames as an array of paths relative to the bldpath
def outFileNames(ctx, bldpath, inFile, **kw):
    zcmgen = ctx.env['ZCMGEN']

    pkgPrefix    = kw.get('pkgPrefix',    '')
    javapkg      = kw.get('javapkg',      'zcmtypes')
    juliapkg     = kw.get('juliapkg',     'zcmtypes')
    juliagenpkgs = kw.get('juliagenpkgs', False)

    lang = kw.get('lang', [])
    if isinstance(lang, str):
        lang = lang.split(' ')

    if (not lang) and (not juliagenpkgs):
        # TODO: this should probably be a more specific error type
        raise WafError('zcmgen requires keword argument: "lang"')

    cmd = {}
    if (pkgPrefix):
        cmd['prefix'] = '--package-prefix %s' % pkgPrefix
    if ('c_stlib' in lang) or ('c_shlib' in lang):
        cmd['c'] = '--c --c-cpath %s --c-hpath %s' % (bldpath, bldpath)
    if 'cpp' in lang:
        cmd['cpp'] = '--cpp --cpp-hpath %s' % (bldpath)
    if 'java' in lang:
        cmd['java'] = '--java --jpath %s --jpkgprefix %s' % (bldpath + '/java', javapkg)
    if 'python' in lang:
        cmd['python'] = '--python --ppath %s' % (bldpath)
    if 'julia' in lang:
        cmd['julia'] = '--julia --julia-path %s' % (bldpath)
        if (juliapkg):
            cmd['julia'] += ' --julia-pkg-prefix %s' % (juliapkg)
        if (juliagenpkgs):
            cmd['julia'] += ' --julia-generate-pkg-files'
    if 'nodejs' in lang:
        cmd['nodejs'] = '--node --npath %s' % (bldpath)

    files = ctx.cmd_and_log('zcm-gen --output-files %s %s' % (' '.join(cmd.values()), inFile),
                            output=waflib.Context.STDOUT,
                            quiet=waflib.Context.BOTH).strip().split()

    for i in range(len(files)):
        if files[i].startswith(bldpath):
            files[i] = files[i][len(bldpath):]
            while (files[i].startswith('/')):
                files[i] = files[i][1:]
        else:
            # Note: just double checking that the output of  `zcmgen --output-files`
            #       is placing files in the proper directory (the build directory)
            raise WafError('ZCMtypes output not in the build path : ', files[i])

    return files

def genJuliaPkgFiles(task):
    gen = task.generator

    zcmgen    = task.env['ZCMGEN']
    bld       = gen.path.get_bld().abspath()

    options = '--julia --julia-path %s --julia-generate-pkg-files' % (bld)

    if (gen.juliapkg):
        options += ' --julia-pkg-prefix %s' % (gen.juliapkg)
    if (gen.pkgPrefix):
        options += ' --package-prefix %s' % (gen.pkgPrefix)

    inputs   = ' '.join([ i.abspath() for i in task.inputs ])

    cmd = '%s %s %s && touch %s' % (zcmgen, options, inputs, task.outputs[0].abspath())

    task.exec_command(cmd)

@conf
def zcmgen(ctx, **kw):
    if not getattr(ctx.env, 'ZCMGEN', []):
        raise WafError('zcmgen requires ctx.env.ZCMGEN set to the zcm-gen executable')

    uselib_name   = kw.get('name',         'zcmtypes')
    building      = kw.get('build',        True)
    pkgPrefix     = kw.get('pkgPrefix',    '')
    littleEndian  = kw.get('littleEndian', False)
    javapkg       = kw.get('javapkg',      'zcmtypes')
    juliapkg      = kw.get('juliapkg',     '')
    juliagenpkgs  = kw.get('juliagenpkgs', False)

    lang = kw.get('lang', [])
    if isinstance(lang, str):
        lang = lang.split(' ')

    if ((not lang) and (not juliagenpkgs)):
        # TODO: this should probably be a more specific error type
        raise WafError('zcmgen requires keword argument: "lang"')

    if 'source' not in kw:
        # TODO: this should probably be a more specific error type
        raise WafError('zcmgen requires keword argument: "source"')

    # exit early if no source files input
    if not kw['source']:
        return

    zcmgen = ctx.root.find_or_declare(ctx.env.ZCMGEN)

    bld = ctx.path.get_bld().abspath()
    inc = os.path.dirname(bld)

    if 'nodejs' in lang:
        bldcmd = '%s --node --npath %s ' % (ctx.env['ZCMGEN'], bld)
        if (pkgPrefix):
            bldcmd += '--package-prefix %s ' % pkgPrefix

        ctx(name   = uselib_name + '_nodejs',
            target = 'zcmtypes.js',
            source = kw['source'],
            rule   = bldcmd + '${SRC}')
        lang.remove('nodejs') # Done with nodejs, remove so we don't try to gen node files below

    if juliagenpkgs:
        ctx(target    = uselib_name + '_juliapkgs',
            rule      = genJuliaPkgFiles,
            source    = kw['source'],
            pkgPrefix = pkgPrefix,
            juliapkg  = juliapkg)

    if len(lang) == 0:
        return

    # Add .zcm files to build so the process_zcmtypes rule picks them up
    genfiles_name = uselib_name + '_genfiles'
    tg = ctx(name         = genfiles_name,
             source       = kw['source'],
             lang         = lang,
             pkgPrefix    = pkgPrefix,
             littleEndian = littleEndian,
             juliapkg     = juliapkg,
             javapkg      = javapkg)
    for s in tg.source:
        ctx.add_manual_dependency(s, zcmgen)

    if not building:
        return

    if 'c_stlib' in lang or 'c_shlib' in lang:
        csrc = []
        for src in tg.source:
            # Note: assuming c_stlib and c_shlib have the same treatment in outFileNames
            files = outFileNames(ctx, ctx.path.get_bld().abspath(), src.abspath(),
                                 lang = [ 'c_stlib', 'c_shlib' ], pkgPrefix = pkgPrefix)
            for f in files:
                if f.endswith('.c'):
                    outnode = ctx.path.find_or_declare(f)
                    csrc.append(outnode)

    if 'c_stlib' in lang:
        ctx.stlib(name            = uselib_name + '_c_stlib',
                  target          = uselib_name,
                  use             = ['default', 'zcm', uselib_name + '_genfiles'],
                  includes        = inc,
                  export_includes = inc,
                  source          = csrc)

    if 'c_shlib' in lang:
        ctx.shlib(name            = uselib_name + '_c_shlib',
                  target          = uselib_name,
                  use             = ['default', 'zcm', uselib_name + '_genfiles'],
                  includes        = inc,
                  export_includes = inc,
                  source          = csrc)

    if 'cpp' in lang:
        ctx(target          = uselib_name + '_cpp',
            rule            = 'touch ${TGT}',
            use             = ['zcm', uselib_name + '_genfiles'],
            export_includes = inc)

    if 'java' in lang:
        ctx(name     = uselib_name + '_java',
            features = 'javac jar',
            use      = ['zcmjar', genfiles_name, uselib_name + '_genfiles'],
            srcdir   = ctx.path.find_or_declare('java/' + javapkg.split('.')[0]),
            outdir   = 'java/classes',  # path to output (for .class)
            basedir  = 'java/classes',  # basedir for jar
            destfile = uselib_name + '.jar')

    if 'python' in lang:
        ctx(target = uselib_name + '_python',
            rule   = 'touch ${TGT}',
            use    = ['zcm', uselib_name + '_genfiles'])

    if 'julia' in lang:
        ctx(target = uselib_name + '_julia',
            rule   = 'touch ${TGT}',
            use    = ['zcm', uselib_name + '_genfiles'])

    ctx(target = uselib_name,
        rule   = 'touch ${TGT}',
        use    = [ 'zcm',
                   uselib_name + '_nodejs',
                   uselib_name + '_juliapkgs',
                   uselib_name + '_c_shlib',
                   uselib_name + '_c_stlib',
                   uselib_name + '_cpp',
                   uselib_name + '_java',
                   uselib_name + '_python',
                   uselib_name + '_julia' ])


@extension('.zcm')
def process_zcmtypes(self, node):
    tsk = self.create_task('zcmgen', node)

class zcmgen(Task.Task):
    color   = 'PINK'
    quiet   = False
    ext_in  = ['.zcm']
    ext_out = ['.c', '.h', '.hpp', '.java', '.py', '.jl']

    ## This method processes the inputs and determines the outputs that will be generated
    ## when the zcm-gen program is run
    def runnable_status(self):
        gen = self.generator
        inp = self.inputs[0]
        bldpath = gen.path.get_bld().abspath()

        files = outFileNames(gen.bld, bldpath, inp.abspath(),
                             pkgPrefix = gen.pkgPrefix,
                             javapkg   = gen.javapkg,
                             juliapkg  = gen.juliapkg,
                             lang      = gen.lang)

        for f in files:
            out_node = gen.path.find_or_declare(f)
            self.outputs.append(out_node)

        if not self.outputs:
            raise WafError('No ZCMtypes generated, ensure a valid lang is specified')

        return super(zcmgen, self).runnable_status()

    ## This method is responsible for producing the files in the self.outputs list
    ## (e.g. actually executing the zcm-gen program)
    def run(self):
        gen = self.generator

        zcmgen = self.env['ZCMGEN']
        zcmfile = self.inputs[0].abspath()
        bld = gen.path.get_bld().abspath()
        inc = bld.split('/')[-1]

        cmd = {}
        if gen.pkgPrefix:
            cmd['prefix'] = '--package-prefix %s' % gen.pkgPrefix
        if gen.littleEndian:
            cmd['endian'] = '--little-endian-encoding'
        if ('c_stlib' in gen.lang) or ('c_shlib' in gen.lang):
            cmd['c'] = '--c --c-typeinfo --c-cpath %s --c-hpath %s --c-include %s' % \
                         (bld, bld, inc)
        if 'cpp' in gen.lang:
            cmd['cpp'] = '--cpp --cpp-hpath %s --cpp-include %s' % (bld, inc)
        if 'java' in gen.lang:
            cmd['java'] = '--java --jpath %s --jpkgprefix %s' % (bld + '/java', gen.javapkg)
        if 'python' in gen.lang:
            cmd['python'] = '--python --ppath %s' % (bld)
        if 'julia' in gen.lang:
            cmd['julia'] = '--julia --julia-path %s' % (bld)
            if (gen.juliapkg):
                cmd['julia'] += ' --julia-pkg-prefix %s' % (gen.juliapkg)

        # no need to check if cmd is empty here, already handled in runnable_status()
        return self.exec_command('%s %s %s' % (zcmgen, zcmfile, ' '.join(cmd.values())))
