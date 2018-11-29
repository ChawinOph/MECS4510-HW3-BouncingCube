@echo off
set MATLAB=C:\PROGRA~1\MATLAB\R2018a
set MATLAB_ARCH=win64
set MATLAB_BIN="C:\Program Files\MATLAB\R2018a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=crossover2pt_gene_mex
set MEX_NAME=crossover2pt_gene_mex
set MEX_EXT=.mexw64
call setEnv.bat
echo # Make settings for crossover2pt_gene > crossover2pt_gene_mex.mki
echo COMPILER=%COMPILER%>> crossover2pt_gene_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> crossover2pt_gene_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> crossover2pt_gene_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> crossover2pt_gene_mex.mki
echo LINKER=%LINKER%>> crossover2pt_gene_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> crossover2pt_gene_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> crossover2pt_gene_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> crossover2pt_gene_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> crossover2pt_gene_mex.mki
echo OMPFLAGS= >> crossover2pt_gene_mex.mki
echo OMPLINKFLAGS= >> crossover2pt_gene_mex.mki
echo EMC_COMPILER=msvc140>> crossover2pt_gene_mex.mki
echo EMC_CONFIG=optim>> crossover2pt_gene_mex.mki
"C:\Program Files\MATLAB\R2018a\bin\win64\gmake" -j 1 -B -f crossover2pt_gene_mex.mk
