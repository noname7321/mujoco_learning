import time
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('../../API-MJCF/force.xml')
d = mujoco.MjData(m)

mujoco.mj_step(m, d)
'''--------box--------'''
box_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")
box_force = [0.0, 0.0, 0.1]
box_torque = [0.0, 0.0, 0.0]
box_point = [0.0, 0.0, 0.0]
red_point = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "red_point")
point = d.site_xpos[red_point]
# mujoco.mj_applyFT(m, d, box_force, box_torque, box_point, box_id, d.qfrc_applied)
'''--------box--------'''

'''--------力——加速度--------'''
sphere_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "sphere")
sphere_force = [0.0, 0.0, 0.0]
sphere_torque = [0.0, 0.0, 0.0]
sphere_point = [0.0, 0.0, 0.0]
# mujoco.mj_applyFT(m, d, sphere_force, sphere_torque, sphere_point, sphere_id, d.qfrc_applied)
'''--------力——加速度--------'''

'''--------扭矩--------'''
pointer_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pointer")
pointer_force = [0.0, 0.0, 0.0]
pointer_torque = [0.0, 0.0, 0.0]
pointer_point = [0.0, 0.0, 0.0]
# mujoco.mj_applyFT(m, d, pointer_force, pointer_torque, pointer_point, pointer_id, d.qfrc_applied)
'''--------扭矩--------'''

with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()
    
    '''--------box--------'''
    # d.qfrc_applied[:] = 0
    # mujoco.mj_applyFT(m, d, box_force, box_torque, point, box_id, d.qfrc_applied)
    # mujoco.mj_step(m, d)
    '''--------box--------'''
    
    '''--------box--------'''
    # box_xfrc_applied = d.xfrc_applied[box_id]
    # box_xfrc_applied[0] = 0.0 # fx
    # box_xfrc_applied[1] = 0.0 # fy
    # box_xfrc_applied[2] = 0.0 # fz
    # box_xfrc_applied[3] = 0.0 # tx
    # box_xfrc_applied[4] = 0.0 # ty
    # box_xfrc_applied[5] = 0.0 # tz
    # mujoco.mj_step(m, d)
    '''--------box--------'''
    
    '''--------力——加速度--------'''
    # d.ctrl[0] = 0.6
    # sphere_xfrc_applied = d.xfrc_applied[sphere_id]
    # sphere_xfrc_applied[0] = 0.0 # fx
    # sphere_xfrc_applied[1] = 0.0 # fy
    # sphere_xfrc_applied[2] = 0.0 # fz
    # sphere_xfrc_applied[3] = 0.0 # tx
    # sphere_xfrc_applied[4] = 0.0 # ty
    # sphere_xfrc_applied[5] = 0.0 # tz
    # mujoco.mj_step(m, d)
    # print("qfrc_passive:%f  qfrc_actuator:%f  qfrc_applied:%f qfrc_bias:%f  efc_force:%f" % (
    # d.qfrc_passive[0], d.qfrc_actuator[0], d.qfrc_applied[0], d.qfrc_bias[0], d.efc_force[0]))
    # print("lin_acc:",d.sensor("lin_acc").data[0])
    # acc = (d.qfrc_passive[0] + d.qfrc_actuator[0] +
    #        d.qfrc_applied[0] + d.qfrc_bias[0] + d.efc_force[0]) / m.body_mass[sphere_id]
    # print("计算加速度:",acc)
    # print("lin_vel:",d.sensor("lin_vel").data[0])
    # print("lin_pos:",d.sensor("lin_pos").data[0])
    '''--------力——加速度--------'''
    
    '''--------扭矩--------'''
    d.ctrl[1] = 0.6
    pointer_xfrc_applied = d.xfrc_applied[pointer_id]
    pointer_xfrc_applied[0] = 0.0 # fx
    pointer_xfrc_applied[1] = 0.0 # fy
    pointer_xfrc_applied[2] = 0.0 # fz
    pointer_xfrc_applied[3] = 0.0 # tx
    pointer_xfrc_applied[4] = 0.0 # ty
    pointer_xfrc_applied[5] = 0.0 # tz
    mujoco.mj_step(m, d)
    print("qfrc_passive:%f  qfrc_actuator:%f  qfrc_applied:%f qfrc_bias:%f  efc_force:%f" % (
      d.qfrc_passive[1], d.qfrc_actuator[1], d.qfrc_applied[1], d.qfrc_bias[1], d.efc_force[1]))
    tau = d.qfrc_passive[1] + d.qfrc_actuator[1] + d.qfrc_applied[1] + d.qfrc_bias[1] + d.efc_force[1]
    print("计算扭矩:",tau)
    print("测量扭矩:",d.sensor("torque").data)
    print("pivot_pos:",d.sensor("pivot_pos").data)
    print("pivot_vel:",d.sensor("pivot_vel").data)
    '''--------扭矩--------'''
    

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)