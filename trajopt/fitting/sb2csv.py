from typing import Union

import pathlib as plb
import re
import json
import os

def sb2csv(inputfilename:plb.Path or str,
           outputfilename:plb.Path or str = "output",
           write_permission:str = "newfile"
           )->[dict,dict]:
    """
    Takes in filename for AVL sb output and returns the A,B matrix components for the 6-dof system
    :param filename:
    :return:
    """

    # check if the file exists
    if type(inputfilename) is str:
        inputfilename = plb.Path(inputfilename)
    if not inputfilename.exists():
        raise FileNotFoundError(f"File {inputfilename} does not exist")

    with open(inputfilename, "rb") as f:
        lines = f.readlines()
        fulltext = f.read()

    # read Amatrix terms (dynamics plant)
    Amatrix_text = dict()
    Amatrix_text["Cx_uvw"] = str(lines[41])
    Amatrix_text["Cy_uvw"] = str(lines[42])
    Amatrix_text["Cz_uvw"] = str(lines[43])
    Amatrix_text["Cx_pqr"] = str(lines[50])
    Amatrix_text["Cy_pqr"] = str(lines[51])
    Amatrix_text["Cz_pqr"] = str(lines[52])

    Amatrix_text["Cl_uvw_txt"] = str(lines[44])
    Amatrix_text["Cm_uvw_txt"] = str(lines[45])
    Amatrix_text["Cn_uvw_txt"] = str(lines[46])
    Amatrix_text["Cl_pqr_txt"] = str(lines[53])
    Amatrix_text["Cm_pqr_txt"] = str(lines[54])
    Amatrix_text["Cn_pqr_txt"] = str(lines[55])

    Amatrix = dict()
    for key,text in Amatrix_text.items():

        terms = re.split('_',key)
        out = re.findall('.?\\d+\\.\\d+',text)
        for idx,entry in enumerate(out):
            Amatrix[f'{terms[0]}{terms[1][idx]}'] = float(entry)

    # read Bmatrix (control plant) terms
    Bmatrix_text = dict()

    #read active control surfaces
    control_surfaces_raw = re.split('\\s+',str(lines[57]))
    control_surface_list = []
    for idx,entry in enumerate(control_surfaces_raw):
        if idx % 2 ==1 and "\\n" not in entry:
            control_surface_list.append(entry)

    # pull out values for bmatrix
    Bmatrix_text["Cx"] = str(lines[59])
    Bmatrix_text["Cy"] = str(lines[60])
    Bmatrix_text["Cz"] = str(lines[61])
    Bmatrix_text["Cl"] = str(lines[62])
    Bmatrix_text["Cm"] = str(lines[63])
    Bmatrix_text["Cn"] = str(lines[64])

    Bmatrix = dict()
    idx = 0
    for key,text in Bmatrix_text.items():
        out = re.findall('.?\\d+\\.\\d+',text)
        for idx,entry in enumerate(out):
            Bmatrix[f'{key}_{control_surface_list[idx]}'] = float(entry)
        idx +=1

    # ====================================================================
    if plb.Path(outputfilename).parent.exists():
        outputfilename = plb.Path(outputfilename)
    else:
        cwd = plb.PosixPath.cwd()
        outputfilename = cwd.joinpath(outputfilename)

    outputdict = dict()
    outputdict["Amatrix"] = Amatrix
    outputdict["Bmatrix"] = Bmatrix

    # if permission is only to make new files check to see if file exists and add suffix
    if write_permission == "newfile":
        idx = 1
        outputfilename_temp = str(outputfilename)
        while plb.Path(outputfilename_temp + ".json").exists():
            outputfilename_temp = str(outputfilename) + "_" + str(idx)
            idx += 1
            if idx > 1000:
                break
        outputfilename = outputfilename_temp + ".json"
    else:
        outputfilename = str(outputfilename) + ".json"

    # save file
    with open(outputfilename, "w") as f:
        json_text = json.dumps(outputdict)
        f.write(json_text)

    return Amatrix, Bmatrix

if __name__ == '__main__':
    filename = "example-files/body-stability-example-ouptut-avl"
    A,B = sb2csv(filename,write_permission="newfile")

    print("Amatrix")
    print(A)
    print("Bmatrix")
    print(B)

    # remove output file
    os.remove('output.json')