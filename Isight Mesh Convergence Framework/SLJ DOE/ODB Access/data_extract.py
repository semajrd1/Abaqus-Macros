from abaqus import *
from abaqusConstants import *
from odbAccess import *
import math
import numpy as np
import __main__
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import optimization
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import sys
import subprocess

odb = openOdb(path="C:\Users\s1232851\Desktop\SLJ DOE\Runtime\Job-1.odb")
char_curve=odb.rootAssembly.instances['COMPOSITE-1'].nodeSets['CHAR_CURVE']
reference_point=odb.rootAssembly.instances['ASSEMBLY'].nodes[0]
NOF=len(odb.steps['Step-1'].frames)
print(NOF)
TT_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['TSAIH'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)

# ============================
A=np.zeros((NOF, TT_length))
frame_TSAIH=[]
for i in range(NOF):
    frame_TSAIH.append(odb.steps['Step-1'].frames[i].fieldOutputs['TSAIH'].getSubset(region=char_curve, position=ELEMENT_NODAL))

max_TSAIH=[]
for i in range(NOF):
    for j in range(0,TT_length):
        A[i][j]=frame_TSAIH[i].values[j].data
    max_TSAIH.append(max(A[i]))
max_max_TSAIH=max_TSAIH[-1]

for i in range(NOF):
    if max_TSAIH[i]>1:
        print('===============TSAI-HILL ANALYSIS===============')
        print('INCREMENT NUMBER:')
        print(i)
        print('LOAD AT FAILURE (N):')
        postFailTSAIH=-odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
        # print(laf)
        print('FAILURE INDEX:')
        print(max_TSAIH[i])
        print('================================================')
        break

# ============================
TT1_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['TSAIW'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)
B=np.zeros((NOF, TT1_length))
frame_TSAIW=[]
for i in range(NOF):
    frame_TSAIW.append(odb.steps['Step-1'].frames[i].fieldOutputs['TSAIW'].getSubset(region=char_curve, position=ELEMENT_NODAL))
# print(frame_TSAIW[1].values[1].data)

max_TSAIW=[]
for i in range(NOF):
    for j in range(0,TT1_length):
        B[i][j]=frame_TSAIW[i].values[j].data
    max_TSAIW.append(max(B[i]))
max_max_TSAIW=max_TSAIW[-1]

for i in range(NOF):
    if max_TSAIW[i]>1:
        print('===============TSAI-WU ANALYSIS===============')
        print('INCREMENT NUMBER:')
        print(i)
        print('LOAD AT FAILURE (N):')
        postFailTSAIW=-odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
        # print(laf)
        print('FAILURE INDEX:')
        print(max_TSAIW[i])
        print('================================================')
        break

# ============================
TT2_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['MSTRS'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)
C=np.zeros((NOF, TT2_length))
frame_MSTRS=[]
for i in range(NOF):
    frame_MSTRS.append(odb.steps['Step-1'].frames[i].fieldOutputs['MSTRS'].getSubset(region=char_curve, position=ELEMENT_NODAL))

max_MSTRS=[]
for i in range(NOF):
    for j in range(0,TT2_length):
        C[i][j]=frame_MSTRS[i].values[j].data
    max_MSTRS.append(max(C[i]))
max_max_MSTRS=max_MSTRS[-1]

for i in range(NOF):
    if max_MSTRS[i]>1:
        print('===============MAX STRESS ANALYSIS===============')
        print('INCREMENT NUMBER:')
        print(i)
        print('LOAD AT FAILURE (N):')
        postFailMSTRS=-odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
        # print(laf)
        print('FAILURE INDEX:')
        print(max_MSTRS[i])
        print('================================================')
        break

# ============================
TT3_length=len(odb.steps['Step-1'].frames[-1].fieldOutputs['AZZIT'].getSubset(region=char_curve, position=ELEMENT_NODAL).values)
D=np.zeros((NOF, TT3_length))
frame_AZZIT=[]
for i in range(NOF):
    frame_AZZIT.append(odb.steps['Step-1'].frames[i].fieldOutputs['AZZIT'].getSubset(region=char_curve, position=ELEMENT_NODAL))

max_AZZIT=[]
for i in range(NOF):
    for j in range(0,TT3_length):
        D[i][j]=frame_AZZIT[i].values[j].data
    max_AZZIT.append(max(D[i]))
max_max_AZZIT=max_AZZIT[-1]

for i in range(NOF):
    if max_AZZIT[i]>1:
        print('===============AZZI-TSAI-HILL ANALYSIS===============')
        print('INCREMENT NUMBER:')
        print(i)
        print('LOAD AT FAILURE (N):')
        postFailAZZIT=-odb.steps['Step-1'].frames[i].fieldOutputs['RF'].getSubset(region=reference_point, position=NODAL).values[0].data[1]
        # print(laf)
        print('FAILURE INDEX:')
        print(max_AZZIT[i])
        print('================================================')
        break

# postFailTSAIW=max_max_TSAIW
# postFailTSAIH=max_max_TSAIH
# postFailMSTRS=max_max_MSTRS
# postFailAZZIT=max_max_AZZIT

pfmaxscrt = str(postFailTSAIW)
pfmaxscrt2 = str(postFailTSAIH)
pfmaxscrt3 = str(postFailMSTRS)
pfquadscrt = str(postFailAZZIT)

wString = 'postFailTSAIW = '+pfmaxscrt+'\n'\
            +'postFailTSAIH = '+pfmaxscrt2+'\n'\
            +'postFailMSTRS = '+pfmaxscrt3+'\n'\
            +'postFailAZZIT = '+pfquadscrt

subprocess.call('IF EXIST "C:\Users\s1232851\Desktop\SLJ DOE\Text\Temp_A_1.txt" DEL /F "C:\Users\s1232851\Desktop\SLJ DOE\Text\Temp_A_1.txt"',shell=True)
f=open(r'C:\Users\s1232851\Desktop\SLJ DOE\Text\Temp_A_1.txt', 'a+')
f.write(wString)
f.close()