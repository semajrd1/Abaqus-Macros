def runUserScript(mdb, values):
    from abaqus import *
    from abaqusConstants import *
    from odbAccess import *
    import csv
    import numpy as np
    import math
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

    DIAMETER = float(values['Diameter'])
    HOLE_ELEM_NUM = int(values['Hole_Elem_Num'])
    LAYUP_J = [float(values['A']),float(values['B']),float(values['C']),float(values['D']),float(values['E']),float(values['F']),float(values['G']),float(values['H'])]

        # COMPOSITE GEOMETRY
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.rectangle(point1=(-3*DIAMETER, -6*DIAMETER -10 -50), point2=(3*DIAMETER, 6*DIAMETER))
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(0.0, DIAMETER/2))
    p = mdb.models['Model-1'].Part(name='Composite', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['Composite']
    p.BaseShell(sketch=s1)
    s1.unsetPrimaryObject()
    
    # PARTITION SKETCH LOCATION
    p = mdb.models['Model-1'].parts['Composite']
    del mdb.models['Model-1'].sketches['__profile__']
    p = mdb.models['Model-1'].parts['Composite']
    f, e, d1 = p.faces, p.edges, p.datums
    t = p.MakeSketchTransform(sketchPlane=f.findAt(coordinates=(0.0, (DIAMETER/2)+1.0, 0.0), normal=(0.0, 0.0, 1.0)), 
        sketchUpEdge=e.findAt(coordinates=(3*DIAMETER, 0.0, 0.0)), sketchPlaneSide=SIDE1, origin=(0.0, 0.0, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=247.38, gridSpacing=6.18, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)

    # PARTITION
    p = mdb.models['Model-1'].parts['Composite']
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.Line(point1=(-3*DIAMETER, -6*DIAMETER -10), point2=(3*DIAMETER, -6*DIAMETER -10))
    s.HorizontalConstraint(entity=g.findAt((0.0, -6*DIAMETER -10)), addUndoState=False)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    pickedFaces = f.findAt(((0.0, (DIAMETER/2)+1.0, 0.0), ))
    e1, d2 = p.edges, p.datums
    p.PartitionFaceBySketch(sketchUpEdge=e1.findAt(coordinates=(3*DIAMETER, 0.0, 0.0)), faces=pickedFaces, sketch=s)
    s.unsetPrimaryObject()

    # PIN GEOMETRY
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(0.0, DIAMETER/2))
    p = mdb.models['Model-1'].Part(name='Pin', dimensionality=THREE_D, type=DISCRETE_RIGID_SURFACE)
    p = mdb.models['Model-1'].parts['Pin']
    p.BaseSolidExtrude(sketch=s, depth=10.0)
    s.unsetPrimaryObject()

    # VTC401-T700 MATERIAL PROPERTIES
    mdb.models['Model-1'].Material(name='VTC401-T700')
    mdb.models['Model-1'].materials['VTC401-T700'].Elastic(type=LAMINA, table=((112479.9, 7230.6, 0.33, 2580.9, 2580.9, 1701.3), ))
    mdb.models['Model-1'].materials['VTC401-T700'].HashinDamageInitiation(table=((2179.9, 810.7, 32.4, 84.1, 68.8, 68.8), ))
    mdb.models['Model-1'].materials['VTC401-T700'].elastic.FailStress(table=((2179.9, 810.7, 32.4, 84.1, 68.9, 0.0, 0.0), ))

    # 4340 STEEL MATERIAL PROPERTIES
    mdb.models['Model-1'].Material(name='steel')
    mdb.models['Model-1'].materials['steel'].Elastic(table=((205000.0, 0.29), ))

    # FACES
    p1 = mdb.models['Model-1'].parts['Composite']
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region = regionToolset.Region(faces=faces)

    # VTC401-T700 MATERIAL PROPERTIES
    p = mdb.models['Model-1'].parts['Composite']
    s = p.faces
    side1Faces = s.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    normalAxisRegion = p.Surface(side1Faces=side1Faces, name='Surf-1')
    mdb.models['Model-1'].parts['Composite'].MaterialOrientation(region=region, 
        orientationType=DISCRETE, axis=AXIS_3, normalAxisDefinition=SURFACE, 
        normalAxisRegion=normalAxisRegion, flipNormalDirection=False, 
        normalAxisDirection=AXIS_3, primaryAxisDefinition=VECTOR, 
        primaryAxisVector=(0.0, 1.0, 0.0), primaryAxisDirection=AXIS_1, 
        flipPrimaryDirection=False, additionalRotationType=ROTATION_NONE, 
        angle=0.0, additionalRotationField='')
    layupOrientation = None

    # COMPOSITE REGIONS
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region1=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region2=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region3=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region4=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region5=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region6=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region7=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    region8=regionToolset.Region(faces=faces)
    p = mdb.models['Model-1'].parts['Composite']
    s = p.faces

    # ASSIGN PLY DEFINITIONS TO COMPOSITE REGIONS
    side1Faces = s.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    normalAxisRegion = p.Surface(side1Faces=side1Faces, name='Surf-2')
    compositeLayup = mdb.models['Model-1'].parts['Composite'].CompositeLayup(
        name='CompositeLayup-1', description='', elementType=SHELL, 
        offsetType=MIDDLE_SURFACE, symmetric=False, 
        thicknessAssignment=FROM_SECTION)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, 
        thicknessType=UNIFORM, poissonDefinition=DEFAULT, temperature=GRADIENT, 
        useDensity=OFF)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-1', region=region1, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[0], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-2', region=region2, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[1], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-3', region=region3, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[2], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-4', region=region4, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[3], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-5', region=region5, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[4], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-6', region=region6, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[5], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-7', region=region7, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[6], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.CompositePly(suppressed=False, plyName='Ply-8', region=region8, 
        material='VTC401-T700', thicknessType=SPECIFY_THICKNESS, thickness=0.2, 
        orientationType=SPECIFY_ORIENT, orientationValue=LAYUP_J[7], 
        additionalRotationType=ROTATION_NONE, additionalRotationField='', 
        axis=AXIS_3, angle=0.0, numIntPoints=3)
    compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, 
        additionalRotationType=ROTATION_NONE, angle=0.0, 
        additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, 
        normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, 
        normalAxisDirection=AXIS_3, flipNormalDirection=False, 
        primaryAxisDefinition=VECTOR, primaryAxisVector=(0.0, 1.0, 0.0), 
        primaryAxisDirection=AXIS_1, flipPrimaryDirection=False)

    # COMPOSITE MESH
    p = mdb.models['Model-1'].parts['Composite']
    p.seedPart(size=1, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models['Model-1'].parts['Composite']
    e = p.edges
    pickedEdges = e.findAt(((-DIAMETER/2, 0.0, 0.0), ))
    p.seedEdgeByNumber(edges=pickedEdges, number=HOLE_ELEM_NUM, constraint=FINER)
    p = mdb.models['Model-1'].parts['Composite']
    p.generateMesh()

    elemType1 = mesh.ElemType(elemCode=S8R, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=STRI65, elemLibrary=STANDARD)
    p = mdb.models['Model-1'].parts['Composite']
    f = p.faces
    faces = f.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    pickedRegions =(faces, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))

    # PIN MESH
    p1 = mdb.models['Model-1'].parts['Pin']
    p = mdb.models['Model-1'].parts['Pin']
    p.seedPart(size=0.25, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models['Model-1'].parts['Pin']
    e = p.edges
    pickedEdges = e.findAt(((DIAMETER/2, 0.0, 10.0), ))
    p.seedEdgeByNumber(edges=pickedEdges, number=HOLE_ELEM_NUM, constraint=FINER)
    p = mdb.models['Model-1'].parts['Pin']
    c2 = p.cells
    p.RemoveCells(cellList=(c2.findAt(coordinates=(0.0, 2.5, 0.0)), ))
    p = mdb.models['Model-1'].parts['Pin']
    p.generateMesh()

    # # PIN SECTION
    # mdb.models['Model-1'].HomogeneousSolidSection(name='Section-1', material='steel', thickness=None)
    # p = mdb.models['Model-1'].parts['Pin']
    # c = p.cells
    # cells = c.findAt(((0.0, 0.0, 0.0), ))
    # region = p.Set(cells=cells, name='Set-2')
    # p = mdb.models['Model-1'].parts['Pin']
    # p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    #     offsetType=MIDDLE_SURFACE, offsetField='', 
    #     thicknessAssignment=FROM_SECTION)

    # ASSEMBLY
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts['Composite']
    a.Instance(name='Composite-1', part=p, dependent=ON)
    a = mdb.models['Model-1'].rootAssembly
    p = mdb.models['Model-1'].parts['Pin']
    a.Instance(name='Pin-1', part=p, dependent=ON)
    a = mdb.models['Model-1'].rootAssembly
    a.translate(instanceList=('Pin-1', ), vector=(0.0, 0.0, -5.0))

    # CONTACT SURFACES
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Composite-1'].faces
    side1Faces1 = s1.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ), ((0.0, (DIAMETER/2)+1.0, 0.0), ))
    a.Surface(side1Faces=side1Faces1, name='Surf-1')
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['Pin-1'].faces
    side1Faces1 = s1.findAt(((-DIAMETER/2, 0.0, -1.0), ))
    a.Surface(side1Faces=side1Faces1, name='Surf-2')

    # STEP-1
    mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial', maxNumInc=10000000, initialInc=0.01, minInc=1e-15)
    mdb.models['Model-1'].steps['Step-1'].control.setValues(allowPropagation=OFF, 
        resetDefaultValues=OFF, timeIncrementation=(4.0, 8.0, 9.0, 16.0, 10.0, 
        4.0, 12.0, 15.0, 6.0, 3.0, 50.0))
    mdb.models['Model-1'].steps['Step-1'].setValues(timeIncrementationMethod=FIXED, noStop=OFF)

    # REFERENCE POINT
    a = mdb.models['Model-1'].rootAssembly
    e11 = a.instances['Composite-1'].edges
    a.ReferencePoint(point=a.instances['Composite-1'].InterestingPoint(edge=e11.findAt(coordinates=(12.0, -6*DIAMETER -10 -50, 0.0)), rule=MIDDLE))

    # INTERACTION
    mdb.models['Model-1'].ContactProperty('IntProp-1')
    mdb.models['Model-1'].interactionProperties['IntProp-1'].TangentialBehavior(
        formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
        pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, 
        table=((0.1, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
        fraction=0.005, elasticSlipStiffness=None)
    mdb.models['Model-1'].interactionProperties['IntProp-1'].NormalBehavior(
        pressureOverclosure=HARD, allowSeparation=ON, 
        constraintEnforcementMethod=DEFAULT)
    mdb.models['Model-1'].StdInitialization(name='CInit-1')
    mdb.models['Model-1'].ContactStd(name='Int-1', createStepName='Initial')
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    r12=mdb.models['Model-1'].rootAssembly.surfaces['Surf-2']
    mdb.models['Model-1'].interactions['Int-1'].includedPairs.setValuesInStep(
        stepName='Initial', useAllstar=OFF, addPairs=((r11, r12), ))
    mdb.models['Model-1'].interactions['Int-1'].contactPropertyAssignments.appendInStep(
        stepName='Initial', assignments=((GLOBAL, SELF, 'IntProp-1'), ))
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    mdb.models['Model-1'].interactions['Int-1'].initializationAssignments.appendInStep(
        stepName='Initial', assignments=((r11, r12, 'CInit-1'), ))
    mdb.models['Model-1'].interactions['Int-1'].surfaceThicknessAssignments.appendInStep(
        stepName='Initial', assignments=((r11, ORIGINAL, 1.0), ))
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    mdb.models['Model-1'].interactions['Int-1'].surfaceOffsetAssignments.appendInStep(
        stepName='Initial', assignments=((r11, ORIGINAL), ))
    r11=mdb.models['Model-1'].rootAssembly.surfaces['Surf-1']
    mdb.models['Model-1'].interactions['Int-1'].surfaceFeatureAssignments.appendInStep(
        stepName='Initial', assignments=((r11, PERIMETER), ))

    # MPC-CONSTRAINT
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.referencePoints
    refPoints1=(r1[r1.keys()[0]], )
    region1=a.Set(referencePoints=refPoints1, name='m_Set-1')
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Composite-1'].faces
    faces1 = f1.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ))
    region2=a.Set(faces=faces1, name='s_Set-1')
    mdb.models['Model-1'].MultipointConstraint(name='Constraint-1', 
        controlPoint=region1, surface=region2, mpcType=TIE_MPC, 
        userMode=DOF_MODE_MPC, userType=0, csys=None)

    # LOADING
    mdb.models['Model-1'].SmoothStepAmplitude(name='Amp-1', timeSpan=STEP, data=((0.0, 0.0), (1.0, 1.0)))
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.referencePoints
    region = a.Set(referencePoints=refPoints1, name='rpset')
    # region = a.Set(referencePoints=refPoints1, name='Set-3')
    # mdb.models['Model-1'].ConcentratedForce(name='Load-1', createStepName='Step-1', 
    #     region=region, cf2=10000.0,  
    #     distributionType=UNIFORM, field='', localCsys=None)
    
    #     a = mdb.models['Model-1'].rootAssembly
    # r1 = a.referencePoints
    # refPoints1=(r1[8], )
    # a.Set(referencePoints=refPoints1, name='rpset')
    # a = mdb.models['Model-1'].rootAssembly
    # r1 = a.referencePoints
    # refPoints1=(r1[8], )
    mdb.models['Model-1'].DisplacementBC(name='BC-disp', createStepName='Step-1', 
        region=region, u1=UNSET, u2=-0.2, u3=UNSET, ur1=UNSET, ur2=UNSET, 
        ur3=UNSET, amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, 
        fieldName='', localCsys=None)


    # PIN BOUNDARY CONDITION
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Pin-1'].faces
    faces1 = f1.findAt(((0.0, 0.0, 5.0), ), ((0.0, 0.0, -5.0), ))
    region = a.Set(faces=faces1, name='Set-7')
    mdb.models['Model-1'].DisplacementBC(name='BC-1', createStepName='Initial', 
        region=region, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
        amplitude=UNSET, distributionType=UNIFORM, fieldName='', 
        localCsys=None)

    # TAB BOUNDARY CONDITION
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances['Composite-1'].faces
    faces1 = f1.findAt(((0.0, -6*DIAMETER -10 -25, 0.0), ))
    region = a.Set(faces=faces1, name='Set-5')
    mdb.models['Model-1'].DisplacementBC(name='BC-2', createStepName='Initial', 
        region=region, u1=SET, u2=UNSET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
        amplitude=UNSET, distributionType=UNIFORM, fieldName='', 
        localCsys=None)

    # FIELD OUTPUT REQUEST
    mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
    'S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'CSTRESS', 'CDISP', 'CFAILURE', 
    'DMICRT', 'HSNFTCRT', 'HSNFCCRT', 'HSNMTCRT', 'HSNMCCRT'))
    
    mdb.models['Model-1'].FieldOutputRequest(name='F-Output-2', 
        createStepName='Step-1', variables=('CFAILURE', 'DMICRT', 'HSNFTCRT', 'HSNFCCRT', 
        'HSNMTCRT', 'HSNMCCRT'), layupNames=('Composite-1.CompositeLayup-1', ), 
        layupLocationMethod=ALL_LOCATIONS, rebar=EXCLUDE)

    # CREATE JOB
    mdb.Job(name='Job-1', model='Model-1', description='', type=ANALYSIS, 
        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=60, 
        memoryUnits=GIGA_BYTES, getMemoryFromAnalysis=True, 
        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
        scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=6, 
        numDomains=6, numGPUs=4)

    # EXTRA REFERENCE POINT FOR RIGID BODY
    p = mdb.models['Model-1'].parts['Pin']
    v2, e1, d2, n1 = p.vertices, p.edges, p.datums, p.nodes
    p.ReferencePoint(point=p.InterestingPoint(edge=e1.findAt(coordinates=(0.0, DIAMETER/2, 10.0)), rule=CENTER))

    # mdb = openMdb(pathName="C:\Users\s1232851\Desktop\Mesh_convergence\Reference\APA5-8mm\CAE_Template")
    D=DIAMETER
    MAG_LAYUP=[]
    for i in range(len(LAYUP_J)):
        MAG_LAYUP.append(math.sqrt(LAYUP_J[i]**2))
    print(MAG_LAYUP)

    APA=sum(MAG_LAYUP)/len(MAG_LAYUP)
    print('APA')
    print(APA)
    PROBE_D=int((99*(D-2))/8)
    print(PROBE_D)
    PROBE_APA=int((APA/90)*99)
    print(PROBE_APA)

    with open('new_char_curve_compression.csv', 'rb') as csvfile:
        char_curve_comp = list(csv.reader(csvfile))
    with open('new_char_curve_tension.csv', 'rb') as csvfile:
        char_curve_tension = list(csv.reader(csvfile))
    d_C=float(char_curve_comp[PROBE_APA][PROBE_D])
    d_T=float(char_curve_tension[PROBE_APA][PROBE_D])
    print('d_C and d_T: direct from matlab')
    print(d_C)
    print(d_T)

    with open('new_ratio_curve_compression.csv', 'rb') as csvfile:
        ratio_curve_comp = list(csv.reader(csvfile))
    with open('new_ratio_curve_tension.csv', 'rb') as csvfile:
        ratio_curve_tension = list(csv.reader(csvfile))
    ratio_C_temp=float(ratio_curve_comp[PROBE_APA][PROBE_D])
    ratio_T_temp=float(ratio_curve_tension[PROBE_APA][PROBE_D])
    KSI_C=math.sqrt((math.sqrt(-ratio_C_temp*(23*ratio_C_temp-24))/ratio_C_temp)-1)/math.sqrt(6)
    KSI_T=math.sqrt((math.sqrt(-ratio_T_temp*(23*ratio_T_temp-24))/ratio_T_temp)-1)/math.sqrt(6)
    d_C=(DIAMETER/2)*((1/KSI_C)-1)
    d_T=(DIAMETER/2)*((1/KSI_T)-1)
    print('d_C and d_T: from matlab ratio surf')
    print(d_C)
    print(d_T)

    LAYUP_J_SQUARED=[]
    for i in range(len(LAYUP_J)):
        LAYUP_J_SQUARED.append(LAYUP_J[i]**2)
    print(LAYUP_J_SQUARED)
    print('ratio-ratio!!')
    RATIO_RATIO=(sum(LAYUP_J_SQUARED)/len(LAYUP_J))/(APA**2)
    print(RATIO_RATIO)
    d_C_adjust=0.8401*RATIO_RATIO**2 - 2.5219*RATIO_RATIO + 2.6833
    if d_C_adjust > 1.0:
        d_C_adjust = 1.0
    d_T_adjust=-0.8583*RATIO_RATIO**2 + 2.5195*RATIO_RATIO - 0.6083
    if d_T_adjust < 1.0:
        d_T_adjust = 1.0
    print('d_C and d_T: adjustment coefficients')
    print(d_C_adjust)
    print(d_T_adjust)

    ratio_C_temp=ratio_C_temp*d_C_adjust
    if ratio_C_temp > 0.99:
        ratio_C_temp = 0.99
    ratio_T_temp=ratio_T_temp*d_T_adjust
    if ratio_T_temp > 0.99:
        ratio_T_temp = 0.99
    print('C and T ratios (respectively):')
    print(ratio_C_temp)
    print(ratio_T_temp)
    KSI_C=math.sqrt((math.sqrt(-ratio_C_temp*(23*ratio_C_temp-24))/ratio_C_temp)-1)/math.sqrt(6)
    KSI_T=math.sqrt((math.sqrt(-ratio_T_temp*(23*ratio_T_temp-24))/ratio_T_temp)-1)/math.sqrt(6)
    d_C=(DIAMETER/2)*((1/KSI_C)-1)
    d_T=(DIAMETER/2)*((1/KSI_T)-1)
    if d_C < 0.0:
        d_C = 0.0
    if d_T < 0.0:
        d_T = 0.0
    print('final d_C and d_T')
    print(d_C)
    print(d_T)
    
    implemented_dist=[]
    char_coord22=[]
    nodes=[]
    global node_labels1
    node_labels1=[]
    node_set=[]

    for i in range(0,21):
        implemented_dist.append(D/2 +d_T +(d_C -d_T)*math.cos(math.radians(i*4.5)))
        char_coord22.append([implemented_dist[i]*math.sin(math.radians(i*4.5)),implemented_dist[i]*math.cos(math.radians(i*4.5))])
        nodes.append(mdb.models['Model-1'].parts['Composite'].nodes.getClosest(coordinates=(((char_coord22[i][0],char_coord22[i][1],0)),)))
        node_labels1.append(nodes[i][0].label)
        str1=str(i*4.5)
    print(node_labels1)
    print(len(node_labels1))
    node_set=mdb.models['Model-1'].parts['Composite'].SetFromNodeLabels('Char_curve', (node_labels1))