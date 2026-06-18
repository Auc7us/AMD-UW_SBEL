# PyChrono model automatically generated using Chrono::SolidWorks add-in
# Assembly: C:\Users\sbel\Downloads\lunarVehicleArm\lunarVehicleArm\Assem2_onlyArm.SLDASM


import pychrono as chrono 
import builtins 

# Some global settings 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'lrv_arm_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0 = chrono.ChBodyAuxRef()
body_0.SetName('SLDW_GROUND')
body_0.SetFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1 = chrono.ChBodyAuxRef()
body_1.SetName('endeffector-1')
body_1.SetPos(chrono.ChVector3d(-2.667,-8.70558992690816e-15,0.325155513123522))
body_1.SetRot(chrono.ChQuaterniond(0.707106781186548,-1.20712009714022e-16,-0.707106781186547,1.99216632648211e-16))
body_1.SetMass(0.626524227569578)
body_1.SetInertiaXX(chrono.ChVector3d(0.0026237186559956,0.000502977063197131,0.00238484258461985))
body_1.SetInertiaXY(chrono.ChVector3d(-9.6347345014623e-19,9.62218303335903e-21,2.8538491491407e-19))
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(7.02339112983057e-18,-1.20943926376326e-17,0.023329601404936),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChVisualShapeModelFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_1)



# Rigid body part
body_2 = chrono.ChBodyAuxRef()
body_2.SetName('bicep-1')
body_2.SetPos(chrono.ChVector3d(-8.07010409222406e-17,1.50119809845186e-16,0.325155513123522))
body_2.SetRot(chrono.ChQuaterniond(-0.5,0.5,-0.5,-0.5))
body_2.SetMass(10.0188345607749)
body_2.SetInertiaXX(chrono.ChVector3d(1.85127451103256,1.82217210560919,0.0315263121628376))
body_2.SetInertiaXY(chrono.ChVector3d(-1.11658003282484e-16,-1.25599415755622e-15,-3.04765594934145e-12))
body_2.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.8504257992554e-14,0.559895724862395,-2.67581948957994e-17),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_2)



# Rigid body part
body_3 = chrono.ChBodyAuxRef()
body_3.SetName('base-1')
body_3.SetPos(chrono.ChVector3d(-3.75783253987686e-62,-2.15904213877361e-78,0.0762000000000001))
body_3.SetRot(chrono.ChQuaterniond(0,-2.77555756156289e-17,1,0))
body_3.SetMass(8.00691853432187)
body_3.SetInertiaXX(chrono.ChVector3d(0.180051635759179,0.180051635759179,0.35121838172002))
body_3.SetInertiaXY(chrono.ChVector3d(-9.44741569709806e-17,-5.5851666924261e-18,3.68087073319269e-18))
body_3.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-6.34810319695294e-17,-2.61923103410295e-17,0.0383731639052751),chrono.ChQuaterniond(1,0,0,0)))
body_3.SetFixed(True)

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_3)



# Rigid body part
body_4 = chrono.ChBodyAuxRef()
body_4.SetName('shoulder-1')
body_4.SetPos(chrono.ChVector3d(-5.74189588383473e-19,7.96390791413929e-18,0.127))
body_4.SetRot(chrono.ChQuaterniond(-2.05721257448836e-17,-0.707106781186548,0.707106781186548,-2.37690812695503e-17))
body_4.SetMass(17.3090829245461)
body_4.SetInertiaXX(chrono.ChVector3d(0.329102703458248,0.359738630712971,0.19573561096493))
body_4.SetInertiaXY(chrono.ChVector3d(-3.87519269175668e-08,-0.000888607344128861,1.45843140654102e-08))
body_4.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(1.41950080889034e-09,-0.000931439058273036,-0.0142042923840082),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChVisualShapeModelFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4.AddVisualShape(body_4_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_4)



# Rigid body part
body_5 = chrono.ChBodyAuxRef()
body_5.SetName('elbow-1')
body_5.SetPos(chrono.ChVector3d(-1.27,-8.49769687811631e-17,0.325155513123522))
body_5.SetRot(chrono.ChQuaterniond(1.54074395550979e-33,2.77555756156289e-17,3.08148791101958e-33,1))
body_5.SetMass(14.504670859222)
body_5.SetInertiaXX(chrono.ChVector3d(0.04629602603643,2.96037800858872,2.94728758334825))
body_5.SetInertiaXY(chrono.ChVector3d(6.40049612112032e-10,4.61982454463747e-08,2.19159727148393e-09))
body_5.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(0.571499998917816,1.58835288481593e-09,5.43151898041859e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChVisualShapeModelFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5.AddVisualShape(body_5_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_5)



# Rigid body part
body_6 = chrono.ChBodyAuxRef()
body_6.SetName('wrist-1')
body_6.SetPos(chrono.ChVector3d(-2.413,-8.23262472967057e-16,0.325155513123523))
body_6.SetRot(chrono.ChQuaterniond(-9.81307786677359e-17,-9.81307786677358e-17,0.707106781186548,0.707106781186547))
body_6.SetMass(1.49908324319661)
body_6.SetInertiaXX(chrono.ChVector3d(0.00191172422781849,0.0108644156852781,0.00979181706223904))
body_6.SetInertiaXY(chrono.ChVector3d(2.61753783284415e-19,-2.6367345673623e-18,-1.35562279998041e-19))
body_6.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(0.10734177092476,-2.26013585595982e-18,4.96127502994239e-18),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_6_1_shape = chrono.ChVisualShapeModelFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6.AddVisualShape(body_6_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_6)



# Rigid body part
body_7 = chrono.ChBodyAuxRef()
body_7.SetName('finger-2')
body_7.SetPos(chrono.ChVector3d(-2.7178,-0.101600000000009,0.325155513123521))
body_7.SetRot(chrono.ChQuaterniond(0.707106781186547,-4.35788199605262e-32,3.92523114670944e-17,0.707106781186548))
body_7.SetMass(0.204687843355227)
body_7.SetInertiaXX(chrono.ChVector3d(0.000246140040298663,0.000981000417461611,0.0012051310562571))
body_7.SetInertiaXY(chrono.ChVector3d(-0.000226453080722204,-1.40752695675067e-19,-7.90586859128437e-21))
body_7.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.0923087540409708,0.0524076781696285,-2.18495338472112e-19),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_7_1_shape = chrono.ChVisualShapeModelFile() 
body_7_1_shape.SetFilename(shapes_dir +'body_7_1.obj') 
body_7.AddVisualShape(body_7_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

contact_material = chrono.ChContactMaterialNSC()
contact_material.SetRollingFriction(0.5)
body_7.AddCollisionShape(chrono.ChCollisionShapeBox(contact_material, 0.005, 0.13, 0.01), chrono.ChFramed(chrono.ChVector3d(-0.106, 0.08, 0), chrono.QUNIT))

body_7.EnableCollision(True)

exported_items.append(body_7)



# Rigid body part
body_8 = chrono.ChBodyAuxRef()
body_8.SetName('finger-1')
body_8.SetPos(chrono.ChVector3d(-2.7178,0.101599999999991,0.325155513123523))
body_8.SetRot(chrono.ChQuaterniond(9.13822863364739e-34,0.707106781186548,-0.707106781186547,3.92523114670944e-17))
body_8.SetMass(0.204687843355227)
body_8.SetInertiaXX(chrono.ChVector3d(0.000246140040298663,0.00098100041746161,0.0012051310562571))
body_8.SetInertiaXY(chrono.ChVector3d(0.000226453080722204,-1.40752695675066e-19,7.90586859128436e-21))
body_8.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.0923087540409708,0.0524076781696285,-2.18495338472112e-19),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_7_1_shape = chrono.ChVisualShapeModelFile() 
body_7_1_shape.SetFilename(shapes_dir +'body_7_1.obj') 
body_8.AddVisualShape(body_7_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

contact_material = chrono.ChContactMaterialNSC()
contact_material.SetRollingFriction(0.5)
body_8.AddCollisionShape(chrono.ChCollisionShapeBox(contact_material, 0.005, 0.13, 0.01), chrono.ChFramed(chrono.ChVector3d(-0.106, 0.08, 0), chrono.QUNIT))

body_8.EnableCollision(True)

exported_items.append(body_8)




# Auxiliary marker (coordinate system feature)
marker_0_1 = chrono.ChMarker()
marker_0_1.SetName('joint_shoulder_base')
body_0.AddMarker(marker_0_1)
marker_0_1.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-5.74189588383473E-19,7.96390791413929E-18,0.127),chrono.ChQuaterniond(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_2 = chrono.ChMarker()
marker_0_2.SetName('joint_bicep_shoulder')
body_0.AddMarker(marker_0_2)
marker_0_2.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-8.07010409222406E-17,1.5234866438078E-16,0.325155513123522),chrono.ChQuaterniond(1.17756934401283E-16,-1.17756934401283E-16,0.707106781186548,-0.707106781186547)))

# Auxiliary marker (coordinate system feature)
marker_0_3 = chrono.ChMarker()
marker_0_3.SetName('joint_elbow_bicep')
body_0.AddMarker(marker_0_3)
marker_0_3.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-1.27,-2.66188598930217E-16,0.325155513123522),chrono.ChQuaterniond(0.707106781186548,-0.707106781186547,-1.17756934401283E-16,1.17756934401283E-16)))

# Auxiliary marker (coordinate system feature)
marker_0_4 = chrono.ChMarker()
marker_0_4.SetName('joint_wrist_elbow')
body_0.AddMarker(marker_0_4)
marker_0_4.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-2.413,-0.0190500000000001,0.325155513123522),chrono.ChQuaterniond(0.707106781186548,-0.707106781186547,-2.17894099802631E-33,2.17894099802631E-33)))

# Auxiliary marker (coordinate system feature)
marker_0_5 = chrono.ChMarker()
marker_0_5.SetName('joint_eff')
body_0.AddMarker(marker_0_5)
marker_0_5.ImposeAbsoluteTransform(chrono.ChFramed(chrono.ChVector3d(-2.6924,-9.00811551237124E-16,0.325155513123523),chrono.ChQuaterniond(-3.92523114670944E-17,3.92523114670943E-17,-0.707106781186547,0.707106781186548)))
