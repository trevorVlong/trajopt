# Created by trevorlong on 7/9/24
# license
# Copyright 2024 trevorlong

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import aerosandbox as asb
import aerosandbox.numpy as np
import pathlib as plb
import pandas as pd
from warnings import warn
from typing import Union


def readPolar(filepath:plb.PosixPath,
              skiprows: Union[int, np.ndarray] = np.arange(0,11)
              ) -> pd.DataFrame:
    """
    Reads an xfoil polar and returns a pandas dataframe of the collected data
    :param filepath: Posix path that points to file of interest
    :param skiprows: number of rows to skip in file before read
    :return:
    """

    # escape if file does not exist
    if not filepath.is_file():
        warn(f"Filepath {str(filepath)} does not exist.")

    dframe = pd.read_table(filepath,
                           skiprows=skiprows,
                           sep="\\s+"  # delimeter is unknown number of spaces
                           )
    # manual add column names because they're always the same
    dframe.columns = ["alpha","CL","CD","CDp","CM","Top_Xtr","Bot_Xtr"]
    return dframe


def read2Dpolar(filelist:Union[plb.PosixPath,list],
                dim2name: str,
                dim2value: Union[float, list],
                mirror:bool=False,
                )->pd.DataFrame:
    """

    :param filelist:
    :return:
    """
    out_df = pd.DataFrame()
    for idx,file in enumerate(filelist):
        df = readPolar(file)
        new_col = dim2value[idx]*np.ones(len(df))
        df.insert(len(df.keys()),dim2name,new_col)
        # concatinate
        out_df = pd.concat([out_df,df])

    # if dataset is specified to be mirrored do it here (use this for symmetric airfoils)
    if mirror:
        orig_df = out_df.copy(deep=True)
        for row in range(len(orig_df)):
            data = orig_df.iloc[row, :]

            # if there is flap deflection perform mirroring by creating new dataframe
            if data["flap_ang"] > 0:
                new_df = pd.DataFrame.from_dict({
                    'alpha': [-data["alpha"]],
                    'CL': [-data["CL"]],
                    'CD': [data["CD"]],
                    'CDp': [data["CDp"]],
                    'CM': [-data["CM"]],
                    'Top_Xtr': [data["Bot_Xtr"]],
                    'Bot_Xtr': [data["Top_Xtr"]],
                    'flap_ang': [-data["flap_ang"]],
                }
                )

                # if first position create df, otherwise concatinate
                if row == 0:
                    out_df = new_df
                else:
                    out_df = pd.concat([out_df, new_df])

    return out_df


if __name__ == "__main__":

    cwd = plb.Path.cwd()

    filepath = plb.Path("/Users/trevorlong/Desktop/control/xfoil_data/naca2412_no_flap")
    filepath2 = plb.Path("/Users/trevorlong/Desktop/control/xfoil_data/naca2412_10deg_080_10")
    dim2name = "flap_ang"
    dim2values = [0,10]

    # attempt multi
    file_list = filepath
    df = read2Dpolar([filepath,filepath2],dim2name,dim2values)
