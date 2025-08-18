# Created by trevorlong on 2/14/25
# license
# Copyright 2025 trevorlong

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import re

import pathlib as plb
from util.openFile import readFile
import warnings

def Run2Geom(runfile:plb.Path,
             geomfile:plb.Path,
             massfile:plb.Path,
             outputfilename:plb.Path = "mass_output.json",
             )->(dict,dict):
    # ======================================================================
    # setup
    physical_param_dict = dict()

    # ======================================================================
    # check if the file exists and read

    runfile_txt,runfile_lines = readFile(runfile)
    geomfile_txt,geomfile_lines = readFile(geomfile)
    massfile_txt, massfile_lines = readFile(massfile)

    # ======================================================================
    # parse
    terms_to_parse = ["Ixx","Iyy","Izz","mass","Sref","Cref","Bref"]

    # ---------------
    # parse massfile (get lunit, munit, tunit)

    massfile_vals = {'Lscale': 0,
                        "Lunit" : '-',
                        "Mscale" : 0,
                        "Munit" : '-',
                        'Tscale' : 0,
                        'Tunit' : '-'}

    for idx,line in enumerate(massfile_lines):

        linestr = str(line)

        if 'Lunit' in linestr and '#' not in linestr:
            temptxts = re.split(' ',linestr)
            massfile_vals["Lunit"] = re.sub('\\n','', temptxts[3])
            massfile_vals["Lscale"] = float(temptxts[2])

        elif 'Tunit' in str(line) and '#' not in linestr:
            temptxts = re.split('\\s+', linestr)
            temptxts = list(filter(None, temptxts))
            massfile_vals["Tunit"] = re.sub('\\n','', temptxts[3])
            massfile_vals["Tscale"] = float(temptxts[2])
        elif 'Munit' in str(line) and '#' not in linestr:
            temptxts = re.split('\\s+',linestr)
            temptxts = list(filter(None, temptxts))
            massfile_vals["Munit"] = re.sub('\\n','', temptxts[3])
            massfile_vals["Mscale"] = float(temptxts[2])
        else:
            continue

        if '-' not in [massfile_vals["Lunit"],massfile_vals["Munit"],massfile_vals["Tunit"]]:
            break
        elif idx == len(massfile_lines):
            warnings.warn("EoF of massfile reached without finding values")

    # ---------------
    # parse .run file
    runfile_vals = dict.fromkeys(["Ixx","Iyy","Izz","mass"])
    for idx, line in enumerate(runfile_lines):
        linestr = str(line)

        if any([key in linestr for key in runfile_vals.keys()]):
            temptxts = re.sub('=','',linestr)
            temptxts = re.split('\\s+', temptxts)
            vals = list(filter(None, temptxts))
            key = vals[0]
            runfile_vals[key] = float(vals[1])
            runfile_vals[f"{key}_units"] = vals[2]

    # ---------------
    # parse .avl file for Sref Cref Bref
    avlfile_vals = dict.fromkeys(["Sref","Bref","Cref"])
    idy = 0
    for idx, line in enumerate(geomfile_lines[1:-1]):

        # break loop if commented line or empty line
        linestr = str(line)
        if "#" in linestr:
            continue
        elif linestr == '\n':
            continue
        elif idy == 2:
            temptxts = re.split('\\s+',linestr)
            vals = list(filter(None, temptxts))
            avlfile_vals['Sref'] = float(vals[0])
            avlfile_vals['Cref'] = float(vals[1])
            avlfile_vals['Bref'] = float(vals[2])

            idy += 1
        else:
            idy += 1

    # ======================================================================
    # package and return as something useful

    # package data in case needed later
    data = {'avl_file_vals': avlfile_vals,
            'runfile_vals' : runfile_vals,
            'massfile_vals': massfile_vals}

    # package massfile vals as something useful
    geomdict = {}

    # mass
    geomdict['mass'] = runfile_vals['mass']
    geomdict['mass_units'] = runfile_vals['mass_units']

    # moment
    geomdict['Ixx'] = runfile_vals['Ixx']
    geomdict['Ixx_units'] = runfile_vals['Ixx_units']
    geomdict['Iyy'] = runfile_vals['Iyy']
    geomdict['Iyy_units'] = runfile_vals['Iyy_units']
    geomdict['Izz'] = runfile_vals['Izz']
    geomdict['Izz_units'] = runfile_vals['Izz_units']

    # reference values
    geomdict['sref'] = avlfile_vals["Sref"]*massfile_vals['Lscale']**2
    geomdict['sref_units'] = f"{massfile_vals['Lunit']}^2"
    geomdict['bref'] = avlfile_vals["Bref"] * massfile_vals['Lscale']
    geomdict['bref_units'] = massfile_vals['Lunit']
    geomdict['cref'] = avlfile_vals["Cref"] * massfile_vals['Lscale']
    geomdict['cref_units'] = massfile_vals['Lunit']



    return geomdict, data
if __name__ == "__main__":
    runfile_ex = plb.Path("example-files/b737.run")
    geomfile_ex = plb.Path("example-files/b737.avl")
    massfile_ex = plb.Path("example-files/b737.mass")
    Run2Geom(runfile_ex,geomfile_ex,massfile_ex)