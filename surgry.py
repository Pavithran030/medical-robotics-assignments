import time, math
import browserbotics as bb

TEAL_DRK = '#1A5C6A'
TEAL_MID = '#2A8898'
TEAL_LT  = '#4AAABB'
STEEL    = '#A8B4BC'
STEEL_DK = '#788890'
SCRN_BLK = '#0A1020'
SCRN_GRN = '#00FF88'
LAMP_WHT = '#FFFFF0'
SKIN_TON = '#E8C4A0'
OR_WALL  = '#C8D4D8'

PEDESTAL_TOP_Z = 0.442
ZERO_QUAT      = [0.0, 0.0, 0.0, 1.0]

home_arm_jpos = [0.0, -0.3, 0.0, -1.8, 0.0, 1.6, 0.8]

JOINT_NAMES = ['J1 Base','J2 Shoulder','J3 Elbow',
               'J4 Elbow2','J5 Wrist1','J6 Wrist2','J7 Wrist3']
JOINT_LO    = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]
JOINT_HI    = [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973]

# Runtime globals — set during init
robot     = None
ee_link   = 10
DOWN_QUAT = None

OBJ_IDS   = {}
OBJ_PICK  = {}
ORIGIN    = {}
BED_DROPS = []

# Arm state
cur_j     = list(home_arm_jpos)   # current joint positions (updated every move)
held_obj  = ''                     # '' = nothing held
drop_idx  = 0


# ════════════════════════════════════════════════════════════
#  WORLD SETUP
# ════════════════════════════════════════════════════════════
def setup_world():
    bb.addGroundPlane()
    BED_X = -0.20
    BED_Y =  0.70

    # Tiled floor
    bb.createBody('box', halfExtent=[4.0,4.0,0.012],
        position=[0,0,0.006], color='#D8E4E8', mass=0)
    for gx in range(-7,8):
        bb.createBody('box', halfExtent=[0.006,4.0,0.002],
            position=[gx*0.5,0,0.013], color='#B8C8CE', mass=0)
    for gy in range(-7,8):
        bb.createBody('box', halfExtent=[4.0,0.006,0.002],
            position=[0,gy*0.5,0.013], color='#B8C8CE', mass=0)

    # Safety zone stripes
    for sx,sy,sw,sd in [
            (BED_X,       BED_Y+0.60, 1.20, 0.025),
            (BED_X,       BED_Y-0.62, 1.20, 0.025),
            (BED_X+1.20,  BED_Y-0.01, 0.025, 0.62),
            (BED_X-1.20,  BED_Y-0.01, 0.025, 0.62)]:
        bb.createBody('box', halfExtent=[sw,sd,0.003],
            position=[sx,sy,0.016], color='#D4A800', mass=0)

    # Walls
    W=4.0; H=2.6; T=0.06; DADO=1.20
    bb.createBody('box', halfExtent=[T,W,DADO/2],
        position=[-W/2,0,DADO/2], color='#8FADB8', mass=0)
    bb.createBody('box', halfExtent=[T,W,(H-DADO)/2],
        position=[-W/2,0,DADO+(H-DADO)/2], color=OR_WALL, mass=0)
    bb.createBody('box', halfExtent=[W,T,DADO/2],
        position=[0,-W/2,DADO/2], color='#8FADB8', mass=0)
    bb.createBody('box', halfExtent=[W,T,(H-DADO)/2],
        position=[0,-W/2,DADO+(H-DADO)/2], color=OR_WALL, mass=0)
    bb.createBody('box', halfExtent=[W*0.35,T,H/2],
        position=[-W/2+W*0.35,W/2,H/2], color=OR_WALL, mass=0)
    bb.createBody('box', halfExtent=[W*0.25,T,H/2],
        position=[W/2-W*0.25,W/2,H/2], color=OR_WALL, mass=0)
    bb.createBody('box', halfExtent=[T+0.004,W,0.028],
        position=[-W/2,0,DADO], color=TEAL_DRK, mass=0)
    bb.createBody('box', halfExtent=[W,T+0.004,0.028],
        position=[0,-W/2,DADO], color=TEAL_DRK, mass=0)
    bb.createBody('box', halfExtent=[T+0.008,W,0.045],
        position=[-W/2,0,0.045], color=TEAL_MID, mass=0)
    bb.createBody('box', halfExtent=[W,T+0.008,0.045],
        position=[0,-W/2,0.045], color=TEAL_MID, mass=0)

    # Ceiling
    bb.createBody('box', halfExtent=[W,W,0.020],
        position=[0,0,H+0.020], color='#F4F6F8', mass=0)
    for gx2 in range(-3,4):
        bb.createBody('box', halfExtent=[0.018,W,0.040],
            position=[gx2*1.0,0,H-0.020], color='#E0E8EC', mass=0)
    for gy2 in range(-3,4):
        bb.createBody('box', halfExtent=[W,0.018,0.040],
            position=[0,gy2*1.0,H-0.020], color='#E0E8EC', mass=0)
    bb.createBody('box', halfExtent=[0.030,1.20,0.022],
        position=[BED_X,BED_Y-0.10,H-0.065], color=STEEL, mass=0)

    # Surgical lamp
    LAZ = H-0.065
    bb.createBody('box', halfExtent=[0.022,0.022,0.40],
        position=[BED_X,BED_Y-0.10,LAZ-0.40], color=STEEL, mass=0)
    bb.createBody('box', halfExtent=[0.18,0.18,0.020],
        position=[BED_X,BED_Y-0.10,LAZ-0.82], color='#D0D8DC', mass=0)
    for lx,ly,lr in [(0,0,0.10),(-0.16,-0.08,0.075),(0.16,-0.08,0.075)]:
        bb.createBody('box', halfExtent=[lr,lr*0.65,0.030],
            position=[BED_X+lx,BED_Y-0.10+ly,LAZ-0.865], color='#C0C8CC', mass=0)
        bb.createBody('box', halfExtent=[lr*0.65,lr,0.030],
            position=[BED_X+lx,BED_Y-0.10+ly,LAZ-0.865], color='#C0C8CC', mass=0)
        bb.createBody('box', halfExtent=[lr*0.70,lr*0.46,0.008],
            position=[BED_X+lx,BED_Y-0.10+ly,LAZ-0.898], color=LAMP_WHT, mass=0)
        bb.createBody('box', halfExtent=[lr*0.46,lr*0.70,0.008],
            position=[BED_X+lx,BED_Y-0.10+ly,LAZ-0.898], color=LAMP_WHT, mass=0)
        for dx2,dy2 in [(-0.03,0),(0.03,0),(0,-0.03),(0,0.03),(0,0),(-0.02,-0.02)]:
            bb.createBody('box', halfExtent=[0.008,0.008,0.004],
                position=[BED_X+lx+dx2,BED_Y-0.10+ly+dy2,LAZ-0.905],
                color='#FFFFF8', mass=0)
    bb.createBody('box', halfExtent=[0.012,0.060,0.012],
        position=[BED_X+0.19,BED_Y-0.10,LAZ-0.86], color=STEEL_DK, mass=0)
    bb.createBody('box', halfExtent=[0.012,0.060,0.012],
        position=[BED_X-0.19,BED_Y-0.10,LAZ-0.86], color=STEEL_DK, mass=0)

    # Operating table
    LEG_H=0.18; BFH=0.05
    bb.createBody('box', halfExtent=[0.080,0.080,LEG_H],
        position=[BED_X,BED_Y,LEG_H], color='#707878', mass=0)
    bb.createBody('box', halfExtent=[0.28,0.18,0.025],
        position=[BED_X,BED_Y,0.025], color='#505858', mass=0)
    for cx,cy in [(-0.22,-0.14),(0.22,-0.14),(-0.22,0.14),(0.22,0.14)]:
        bb.createBody('box', halfExtent=[0.030,0.018,0.022],
            position=[BED_X+cx,BED_Y+cy,0.022], color='#202828', mass=0)
    TABLE_Z = LEG_H*2+BFH
    bb.createBody('box', halfExtent=[0.90,0.40,BFH],
        position=[BED_X,BED_Y,TABLE_Z], color='#909898', mass=0)
    for dy in [-0.40,0.40]:
        bb.createBody('box', halfExtent=[0.90,0.015,BFH+0.018],
            position=[BED_X,BED_Y+dy,TABLE_Z], color=STEEL, mass=0)
    for dx in [-0.89,0.89]:
        bb.createBody('box', halfExtent=[0.015,0.40,BFH+0.018],
            position=[BED_X+dx,BED_Y,TABLE_Z], color=STEEL, mass=0)
    for dx2 in [-0.60,-0.20,0.20,0.60]:
        for dy2 in [-0.40,0.40]:
            bb.createBody('box', halfExtent=[0.020,0.020,0.018],
                position=[BED_X+dx2,BED_Y+dy2,TABLE_Z+BFH+0.009],
                color=TEAL_MID, mass=0)
    MATT_Z = TABLE_Z+BFH*2+0.025
    bb.createBody('box', halfExtent=[0.87,0.37,0.025],
        position=[BED_X,BED_Y,MATT_Z-0.012], color='#1A6A78', mass=0)
    bb.createBody('box', halfExtent=[0.86,0.36,0.032],
        position=[BED_X,BED_Y,MATT_Z+0.010], color='#1E7888', mass=0)
    DRAPE_Z = MATT_Z+0.045
    bb.createBody('box', halfExtent=[0.84,0.34,0.006],
        position=[BED_X,BED_Y,DRAPE_Z], color='#2E6888', mass=0)
    for dx3 in [-0.40,0,0.40]:
        bb.createBody('box', halfExtent=[0.004,0.33,0.003],
            position=[BED_X+dx3,BED_Y,DRAPE_Z+0.007], color='#265A78', mass=0)
    PIL_X = BED_X-0.68
    bb.createBody('box', halfExtent=[0.14,0.28,0.055],
        position=[PIL_X,BED_Y,DRAPE_Z+0.055], color='#F8FAFA', mass=0)
    bb.createBody('box', halfExtent=[0.095,0.080,0.090],
        position=[PIL_X+0.02,BED_Y,DRAPE_Z+0.14], color=SKIN_TON, mass=0)
    bb.createBody('box', halfExtent=[0.040,0.055,0.020],
        position=[PIL_X+0.10,BED_Y,DRAPE_Z+0.155], color='#90C8B0', mass=0)
    for i in range(5):
        bb.createBody('box', halfExtent=[0.008,0.010,0.010],
            position=[PIL_X+0.14+i*0.025,BED_Y-0.06,DRAPE_Z+0.16],
            color='#A0B8A8', mass=0)
    bb.createBody('box', halfExtent=[0.010,0.30,0.012],
        position=[PIL_X-0.16,BED_Y,DRAPE_Z+0.18], color=STEEL, mass=0)

    # Anaesthesia machine
    AM_X=-1.30; AM_Y=-0.80
    bb.createBody('box', halfExtent=[0.22,0.18,0.65],
        position=[AM_X,AM_Y,0.65], color='#D0D8DC', mass=0)
    bb.createBody('box', halfExtent=[0.010,0.16,0.55],
        position=[AM_X+0.22,AM_Y,0.68], color='#2A3038', mass=0)
    bb.createBody('box', halfExtent=[0.008,0.12,0.10],
        position=[AM_X+0.228,AM_Y,0.90], color=SCRN_BLK, mass=0)
    for wi,wc in enumerate([SCRN_GRN,'#FF6060','#60C0FF']):
        bb.createBody('box', halfExtent=[0.006,0.10,0.005],
            position=[AM_X+0.232,AM_Y,0.96-wi*0.06], color=wc, mass=0)
    for ky in [-0.06,-0.02,0.02,0.06]:
        bb.createBody('box', halfExtent=[0.012,0.012,0.012],
            position=[AM_X+0.228,AM_Y+ky,0.72], color='#707880', mass=0)
    for li,lc in enumerate(['#00FF00','#FFAA00','#FF3030','#00AAFF']):
        bb.createBody('box', halfExtent=[0.006,0.006,0.004],
            position=[AM_X+0.232,AM_Y-0.08+li*0.055,0.62], color=lc, mass=0)
    for vy,vc in [(-0.06,'#C0A040'),(0.06,'#40A0C0')]:
        bb.createBody('box', halfExtent=[0.022,0.022,0.08],
            position=[AM_X+0.20,AM_Y+vy,1.20], color=vc, mass=0)
        bb.createBody('box', halfExtent=[0.016,0.016,0.012],
            position=[AM_X+0.20,AM_Y+vy,1.292], color='#808080', mass=0)
    for cdy,cc in [(-0.10,'#20A840'),(0.10,'#2060C0')]:
        bb.createBody('box', halfExtent=[0.040,0.040,0.35],
            position=[AM_X-0.06,AM_Y+cdy,0.38], color=cc, mass=0)
        bb.createBody('box', halfExtent=[0.028,0.028,0.060],
            position=[AM_X-0.06,AM_Y+cdy,0.790], color='#C0C0C0', mass=0)
        bb.createBody('box', halfExtent=[0.015,0.015,0.020],
            position=[AM_X-0.06,AM_Y+cdy,0.870], color='#808080', mass=0)
    for i in range(8):
        bb.createBody('box', halfExtent=[0.012,0.012,0.008],
            position=[AM_X+0.22,AM_Y,1.10+i*0.022], color='#80B098', mass=0)

    # Vital signs monitor
    VM_X=-1.88; VM_Y=0.40; VM_Z=1.40
    bb.createBody('box', halfExtent=[0.012,0.080,0.060],
        position=[VM_X+0.012,VM_Y,VM_Z], color=STEEL, mass=0)
    bb.createBody('box', halfExtent=[0.060,0.080,0.012],
        position=[VM_X+0.060,VM_Y,VM_Z+0.048], color=STEEL, mass=0)
    bb.createBody('box', halfExtent=[0.012,0.20,0.14],
        position=[VM_X+0.120,VM_Y,VM_Z], color='#1A2028', mass=0)
    bb.createBody('box', halfExtent=[0.008,0.18,0.12],
        position=[VM_X+0.130,VM_Y,VM_Z], color=SCRN_BLK, mass=0)
    for wi2,wc2 in enumerate([SCRN_GRN,'#FF8060','#60C0FF','#FFDD00']):
        bb.createBody('box', halfExtent=[0.006,0.16,0.004],
            position=[VM_X+0.136,VM_Y,VM_Z+0.075-wi2*0.045], color=wc2, mass=0)
        bb.createBody('box', halfExtent=[0.006,0.030,0.016],
            position=[VM_X+0.136,VM_Y+0.145,VM_Z+0.075-wi2*0.045], color=wc2, mass=0)
    bb.createBody('box', halfExtent=[0.008,0.014,0.014],
        position=[VM_X+0.132,VM_Y-0.175,VM_Z+0.110], color='#FF4040', mass=0)

    # Scrub nurse table
    ST_X=0.60; ST_Y=-1.10
    for stlx,stly in [(-0.30,-0.20),(0.30,-0.20),(-0.30,0.20),(0.30,0.20)]:
        bb.createBody('box', halfExtent=[0.014,0.014,0.45],
            position=[ST_X+stlx,ST_Y+stly,0.45], color='#A0A8B0', mass=0)
        bb.createBody('box', halfExtent=[0.022,0.022,0.012],
            position=[ST_X+stlx,ST_Y+stly,0.012], color='#505860', mass=0)
    bb.createBody('box', halfExtent=[0.32,0.22,0.010],
        position=[ST_X,ST_Y,0.910], color='#C0CCD4', mass=0)
    for sex,sey,sew,sed in [
            (0.32,0,0.010,0.22),(-0.32,0,0.010,0.22),
            (0,0.22,0.32,0.010),(0,-0.22,0.32,0.010)]:
        bb.createBody('box', halfExtent=[sew,sed,0.022],
            position=[ST_X+sex,ST_Y+sey,0.922], color='#909AA0', mass=0)
    bb.createBody('box', halfExtent=[0.30,0.20,0.003],
        position=[ST_X,ST_Y,0.924], color='#3070A0', mass=0)

    # Crash cart
    CC_X=1.20; CC_Y=-1.20
    bb.createBody('box', halfExtent=[0.18,0.12,0.44],
        position=[CC_X,CC_Y,0.44], color='#E04020', mass=0)
    for di in range(4):
        dz=0.08+di*0.18
        bb.createBody('box', halfExtent=[0.010,0.10,0.070],
            position=[CC_X+0.18,CC_Y,dz+0.035], color='#C03010', mass=0)
        bb.createBody('box', halfExtent=[0.014,0.040,0.010],
            position=[CC_X+0.188,CC_Y,dz+0.035], color='#D0D8DC', mass=0)
    bb.createBody('box', halfExtent=[0.12,0.09,0.060],
        position=[CC_X,CC_Y,0.940], color='#202830', mass=0)
    bb.createBody('box', halfExtent=[0.095,0.070,0.008],
        position=[CC_X,CC_Y,0.998], color=SCRN_BLK, mass=0)
    bb.createBody('box', halfExtent=[0.080,0.055,0.004],
        position=[CC_X,CC_Y,1.004], color='#10C840', mass=0)

    # IV pole
    IV_X=BED_X-1.00; IV_Y=BED_Y+0.42
    bb.createBody('box', halfExtent=[0.025,0.015,0.014],
        position=[IV_X,IV_Y,0.014], color='#404848', mass=0)
    bb.createBody('box', halfExtent=[0.008,0.008,0.90],
        position=[IV_X,IV_Y,0.90], color='#B0B8C0', mass=0)
    bb.createBody('box', halfExtent=[0.060,0.006,0.006],
        position=[IV_X,IV_Y,1.78], color='#9098A0', mass=0)
    for hx in [-0.055,0.055]:
        bb.createBody('box', halfExtent=[0.006,0.006,0.025],
            position=[IV_X+hx,IV_Y,1.808], color='#C0C8D0', mass=0)
    bb.createBody('box', halfExtent=[0.040,0.010,0.065],
        position=[IV_X,IV_Y,1.720], color='#C8E8F0', mass=0)
    bb.createBody('box', halfExtent=[0.032,0.008,0.050],
        position=[IV_X,IV_Y,1.720], color='#E0F4F8', mass=0)
    for i in range(8):
        bb.createBody('box', halfExtent=[0.003,0.003,0.020],
            position=[IV_X+0.010,IV_Y,1.640-i*0.045], color='#80B0B8', mass=0)
    bb.createBody('box', halfExtent=[0.055,0.035,0.070],
        position=[IV_X,IV_Y,1.38], color='#E8EEF2', mass=0)
    bb.createBody('box', halfExtent=[0.010,0.028,0.055],
        position=[IV_X+0.055,IV_Y,1.38], color='#1A2830', mass=0)
    bb.createBody('box', halfExtent=[0.008,0.022,0.030],
        position=[IV_X+0.062,IV_Y,1.395], color=SCRN_BLK, mass=0)
    bb.createBody('box', halfExtent=[0.006,0.018,0.022],
        position=[IV_X+0.066,IV_Y,1.395], color='#00C860', mass=0)

    # Wall clock
    CK_X=-1.94; CK_Y=0.0; CK_Z=1.85
    bb.createBody('box', halfExtent=[0.012,0.095,0.095],
        position=[CK_X,CK_Y,CK_Z], color='#E8ECEE', mass=0)
    bb.createBody('box', halfExtent=[0.008,0.082,0.082],
        position=[CK_X+0.010,CK_Y,CK_Z], color='#F8FAFA', mass=0)
    for hm in range(12):
        ang = hm*math.pi/6
        bb.createBody('box', halfExtent=[0.004,0.004,0.006],
            position=[CK_X+0.016,
                      CK_Y+0.068*math.sin(ang),
                      CK_Z+0.068*math.cos(ang)],
            color='#303838', mass=0)
    bb.createBody('box', halfExtent=[0.003,0.003,0.035],
        position=[CK_X+0.016,CK_Y+0.016,CK_Z+0.012], color='#202828', mass=0)
    bb.createBody('box', halfExtent=[0.003,0.003,0.050],
        position=[CK_X+0.016,CK_Y-0.022,CK_Z+0.020], color='#303838', mass=0)
    bb.createBody('box', halfExtent=[0.002,0.002,0.048],
        position=[CK_X+0.016,CK_Y+0.028,CK_Z-0.018], color='#E04020', mass=0)

    # Robot pedestal
    PED_X=BED_X+0.10; PED_Y=BED_Y-0.62
    bb.createBody('box', halfExtent=[0.24,0.24,0.030],
        position=[PED_X,PED_Y,0.030], color='#2A3540', mass=0)
    for bx,by in [(-0.18,-0.18),(0.18,-0.18),(-0.18,0.18),(0.18,0.18)]:
        bb.createBody('box', halfExtent=[0.016,0.016,0.014],
            position=[PED_X+bx,PED_Y+by,0.074], color=TEAL_DRK, mass=0)
        bb.createBody('box', halfExtent=[0.008,0.008,0.008],
            position=[PED_X+bx,PED_Y+by,0.096], color='#607080', mass=0)
    for zh in range(6):
        z=0.062+zh*0.032; r=0.102
        col=TEAL_MID if zh%2==0 else '#505E6A'
        bb.createBody('box', halfExtent=[r,r*0.42,0.016],
            position=[PED_X,PED_Y,z], color=col, mass=0)
        bb.createBody('box', halfExtent=[r*0.42,r,0.016],
            position=[PED_X,PED_Y,z], color=col, mass=0)
    bb.createBody('box', halfExtent=[0.016,0.010,0.12],
        position=[PED_X+0.105,PED_Y,0.062+0.12], color='#3A4A54', mass=0)
    for si in range(3):
        bb.createBody('box', halfExtent=[0.004,0.004,0.004],
            position=[PED_X+0.105,PED_Y+0.005,0.10+si*0.035],
            color=TEAL_LT, mass=0)
    lower_top = 0.062+6*0.032
    bb.createBody('box', halfExtent=[0.138,0.138,0.024],
        position=[PED_X,PED_Y,lower_top+0.024], color='#2A3540', mass=0)
    up_z = lower_top+0.048
    for zh2 in range(3):
        z2=up_z+zh2*0.026; r2=0.085
        bb.createBody('box', halfExtent=[r2,r2*0.42,0.013],
            position=[PED_X,PED_Y,z2], color='#505E6A', mass=0)
        bb.createBody('box', halfExtent=[r2*0.42,r2,0.013],
            position=[PED_X,PED_Y,z2], color='#4A5860', mass=0)
    top_cap_c = up_z+3*0.026+0.020
    bb.createBody('box', halfExtent=[0.128,0.128,0.020],
        position=[PED_X,PED_Y,top_cap_c], color='#2A3540', mass=0)

    # Instrument tray
    TRAY_X=PED_X+0.58; TRAY_Y=PED_Y; TRAY_H=0.76
    for tlx,tly in [(-0.28,-0.18),(0.28,-0.18),(-0.28,0.18),(0.28,0.18)]:
        bb.createBody('box', halfExtent=[0.013,0.013,TRAY_H/2],
            position=[TRAY_X+tlx,TRAY_Y+tly,TRAY_H/2], color='#A0A8B0', mass=0)
        bb.createBody('box', halfExtent=[0.020,0.020,0.014],
            position=[TRAY_X+tlx,TRAY_Y+tly,TRAY_H*0.55], color=TEAL_MID, mass=0)
        bb.createBody('box', halfExtent=[0.006,0.006,0.006],
            position=[TRAY_X+tlx+0.018,TRAY_Y+tly,TRAY_H*0.55],
            color='#D0D8DC', mass=0)
        bb.createBody('box', halfExtent=[0.022,0.012,0.016],
            position=[TRAY_X+tlx,TRAY_Y+tly,0.016], color='#181820', mass=0)
    for tly2 in [-0.18,0.18]:
        bb.createBody('box', halfExtent=[0.28,0.008,0.008],
            position=[TRAY_X,TRAY_Y+tly2,TRAY_H*0.42], color='#909AA0', mass=0)
    for tlx2 in [-0.28,0.28]:
        bb.createBody('box', halfExtent=[0.008,0.18,0.008],
            position=[TRAY_X+tlx2,TRAY_Y,TRAY_H*0.42], color='#909AA0', mass=0)
    bb.createBody('box', halfExtent=[0.27,0.17,0.007],
        position=[TRAY_X,TRAY_Y,TRAY_H*0.42-0.038], color='#B8C4CC', mass=0)
    bb.createBody('box', halfExtent=[0.32,0.21,0.009],
        position=[TRAY_X,TRAY_Y,TRAY_H+0.009], color='#C0CCD4', mass=0)
    for tex,tey,tew,ted in [
            (0.32,0,0.011,0.21),(-0.32,0,0.011,0.21),
            (0,0.21,0.32,0.011),(0,-0.21,0.32,0.011)]:
        bb.createBody('box', halfExtent=[tew,ted,0.028],
            position=[TRAY_X+tex,TRAY_Y+tey,TRAY_H+0.028], color='#A8B4BC', mass=0)
    bb.createBody('box', halfExtent=[0.30,0.19,0.004],
        position=[TRAY_X,TRAY_Y,TRAY_H+0.022], color='#2A6888', mass=0)
    for ddx in [-0.15,0.0,0.15]:
        bb.createBody('box', halfExtent=[0.003,0.18,0.002],
            position=[TRAY_X+ddx,TRAY_Y,TRAY_H+0.025], color='#225878', mass=0)

    OBJ_Z = TRAY_H+0.026

    # Scalpel decorative
    SC_X=TRAY_X-0.15; SC_Y=TRAY_Y
    for i in range(7):
        bb.createBody('box', halfExtent=[0.008,0.040-i*0.001,0.008],
            position=[SC_X,SC_Y+0.020-i*0.014,OBJ_Z+0.009],
            color='#C8D0D8', mass=0)
    for i in range(4):
        bb.createBody('box', halfExtent=[0.009,0.002,0.009],
            position=[SC_X,SC_Y-0.010-i*0.014,OBJ_Z+0.009],
            color='#A0A8B0', mass=0)
    bb.createBody('box', halfExtent=[0.004,0.030,0.004],
        position=[SC_X,SC_Y+0.105,OBJ_Z+0.005], color='#E0E8F0', mass=0)
    bb.createBody('box', halfExtent=[0.003,0.020,0.002],
        position=[SC_X+0.002,SC_Y+0.120,OBJ_Z+0.003], color='#F0F8FF', mass=0)

    # Forceps decorative
    FC_X=TRAY_X; FC_Y=TRAY_Y
    for i in range(10):
        bb.createBody('box', halfExtent=[0.005,0.005,0.006],
            position=[FC_X-0.006,FC_Y-0.050+i*0.018,OBJ_Z+0.006],
            color='#C0C8D0', mass=0)
        bb.createBody('box', halfExtent=[0.005,0.005,0.006],
            position=[FC_X+0.006,FC_Y-0.050+i*0.018,OBJ_Z+0.006],
            color='#C8D0D8', mass=0)
    bb.createBody('box', halfExtent=[0.003,0.022,0.004],
        position=[FC_X,FC_Y+0.132,OBJ_Z+0.006], color='#D0D8E0', mass=0)
    for ry in [-0.060,-0.072]:
        bb.createBody('box', halfExtent=[0.016,0.004,0.016],
            position=[FC_X,FC_Y+ry,OBJ_Z+0.016], color='#B8C0C8', mass=0)
        bb.createBody('box', halfExtent=[0.004,0.004,0.016],
            position=[FC_X-0.012,FC_Y+ry,OBJ_Z+0.016], color='#B8C0C8', mass=0)
        bb.createBody('box', halfExtent=[0.004,0.004,0.016],
            position=[FC_X+0.012,FC_Y+ry,OBJ_Z+0.016], color='#B8C0C8', mass=0)
    bb.createBody('box', halfExtent=[0.010,0.004,0.006],
        position=[FC_X,FC_Y-0.020,OBJ_Z+0.008], color='#A0A8B0', mass=0)

    # Suture decorative
    SK_X=TRAY_X+0.15; SK_Y=TRAY_Y
    bb.createBody('box', halfExtent=[0.045,0.055,0.010],
        position=[SK_X,SK_Y,OBJ_Z+0.010], color='#E8E0C8', mass=0)
    for fi in range(4):
        bb.createBody('box', halfExtent=[0.044,0.003,0.002],
            position=[SK_X,SK_Y-0.035+fi*0.024,OBJ_Z+0.021],
            color='#D8D0B8', mass=0)
    bb.createBody('box', halfExtent=[0.030,0.030,0.002],
        position=[SK_X,SK_Y,OBJ_Z+0.022], color='#3060A0', mass=0)
    for li2 in range(3):
        bb.createBody('box', halfExtent=[0.025,0.004,0.001],
            position=[SK_X,SK_Y+0.010-li2*0.010,OBJ_Z+0.024],
            color='#FFFFFF', mass=0)

    for mx3,mc3 in [(SC_X,'#C0C8D8'),(FC_X,'#8090A0'),(SK_X,'#3060A0')]:
        bb.createBody('box', halfExtent=[0.010,0.010,0.005],
            position=[mx3,TRAY_Y-0.175,TRAY_H+0.038], color=mc3, mass=0)
    bb.createBody('box', halfExtent=[0.070,0.005,0.022],
        position=[TRAY_X,TRAY_Y-0.205,TRAY_H+0.040], color='#FFFFFF', mass=0)
    bb.createBody('box', halfExtent=[0.065,0.004,0.007],
        position=[TRAY_X,TRAY_Y-0.205,TRAY_H+0.054], color=TEAL_DRK, mass=0)

    return BED_X, BED_Y, TRAY_X, TRAY_Y, TRAY_H, OBJ_Z, SC_X, FC_X, SK_X


# ════════════════════════════════════════════════════════════
#  INIT
# ════════════════════════════════════════════════════════════
bb.setGravity(0, 0, 0)
DOWN_QUAT = bb.getQuaternionFromEuler([math.pi, 0, 0])

BED_X, BED_Y, TRAY_X, TRAY_Y, TRAY_H, OBJ_Z, SC_X, FC_X, SK_X = setup_world()

PED_X = BED_X+0.10
PED_Y = BED_Y-0.58

robot = bb.loadURDF('panda.urdf', [PED_X, PED_Y, PEDESTAL_TOP_Z],
            bb.getQuaternionFromEuler([0, 0, math.pi/2]), fixedBase=True)
ee_link = 10

for i, jp in enumerate(home_arm_jpos):
    bb.setJointMotorControl(robot, i, targetPosition=jp)
    cur_j[i] = jp

# Pickable proxy objects
scalpel_id = bb.createBody('box', halfExtent=[0.010,0.075,0.010],
    position=[SC_X, TRAY_Y, OBJ_Z+0.010], color='#C8D0D8', mass=0)
forceps_id = bb.createBody('box', halfExtent=[0.010,0.095,0.010],
    position=[FC_X, TRAY_Y, OBJ_Z+0.010], color='#8090A0', mass=0)
suture_id  = bb.createBody('box', halfExtent=[0.048,0.058,0.012],
    position=[SK_X, TRAY_Y, OBJ_Z+0.012], color='#3060A0', mass=0)

OBJ_IDS  = {'scalpel': scalpel_id, 'forceps': forceps_id, 'suture': suture_id}
OBJ_PICK = {
    'scalpel': [SC_X, TRAY_Y, OBJ_Z+0.015],
    'forceps':  [FC_X, TRAY_Y, OBJ_Z+0.015],
    'suture':   [SK_X, TRAY_Y, OBJ_Z+0.016],
}
ORIGIN = {
    'scalpel': [SC_X,  TRAY_Y, OBJ_Z+0.010],
    'forceps':  [FC_X, TRAY_Y, OBJ_Z+0.010],
    'suture':   [SK_X, TRAY_Y, OBJ_Z+0.012],
}
BED_DROPS = [
    [BED_X+0.20, BED_Y-0.15, 0.80],
    [BED_X+0.20, BED_Y+0.00, 0.80],
    [BED_X+0.20, BED_Y+0.15, 0.80],
]

for jname,jdef,jlo,jhi in zip(JOINT_NAMES, home_arm_jpos, JOINT_LO, JOINT_HI):
    bb.addDebugSlider(jname, jdef, jlo, jhi)

bb.addDebugButton('Pick Scalpel')
bb.addDebugButton('Pick Forceps')
bb.addDebugButton('Pick Suture')
bb.addDebugButton('Drop on Table')
bb.addDebugButton('Return Home')
bb.addDebugButton('Reset All')

old_vals = {
    'Pick Scalpel':0,'Pick Forceps':0,'Pick Suture':0,
    'Drop on Table':0,'Return Home':0,'Reset All':0
}


# ════════════════════════════════════════════════════════════
#  HELPERS
# ════════════════════════════════════════════════════════════

def get_ee_pos():
    try:
        return list(bb.getLinkState(robot, ee_link)[0])
    except Exception:
        return [0.0, 0.0, 1.0]

def ik(pos):
    return list(bb.calculateInverseKinematics(robot, ee_link, pos, DOWN_QUAT))

def ease(t):
    t = max(0.0, min(1.0, t))
    return t*t*(3.0-2.0*t)

def snap_to_ee(obj_name):
    """Teleport named object exactly to current EE position."""
    if obj_name and obj_name in OBJ_IDS:
        try:
            bb.resetBasePositionAndOrientation(
                OBJ_IDS[obj_name], get_ee_pos(), ZERO_QUAT)
        except Exception:
            pass

def place_obj(obj_name, pos):
    """Teleport named object to an explicit position."""
    if obj_name and obj_name in OBJ_IDS:
        try:
            bb.resetBasePositionAndOrientation(
                OBJ_IDS[obj_name],
                [float(pos[0]), float(pos[1]), float(pos[2])],
                ZERO_QUAT)
        except Exception:
            pass


def smooth_move(target_j, steps=60, carrying=False, carry_obj=''):
    """
    ══ CORE MOTION — identical pattern to reference smooth_joints() ══

    Blocking loop: interpolates cur_j → target_j over `steps` frames.
    On EVERY frame, if carrying=True, snaps carry_obj to EE.
    This is the same guarantee as the reference rover code's
    snap_knife_to_gripper() called inside smooth_joints().

    No threading needed — the main loop calls this directly and
    waits for it to return, exactly like the reference code does.
    """
    global cur_j
    start_j = list(cur_j)

    for s in range(steps + 1):
        t  = s / max(steps, 1)
        t2 = ease(t)
        for i in range(7):
            v = start_j[i] + (target_j[i] - start_j[i]) * t2
            bb.setJointMotorControl(robot, i, targetPosition=float(v))
            cur_j[i] = float(v)

        # ── KEY: weld object to EE on every frame ──
        if carrying and carry_obj:
            snap_to_ee(carry_obj)

        time.sleep(0.018)   # ~55 fps — same as reference 0.02 s


# ════════════════════════════════════════════════════════════
#  PICK / DROP SEQUENCES  (blocking, called directly like reference)
# ════════════════════════════════════════════════════════════

def do_pick(obj):
    """
    Full pick sequence — mirrors reference pick_and_deliver():
      1. Move to hover above object  (not yet carrying)
      2. Descend onto object         (snap from frame 1)
      3. Lift up                     (carrying every frame)
      4. Return to home pose         (carrying every frame)
    Returns when arm is at home holding the object.
    """
    global held_obj

    pick  = OBJ_PICK[obj]
    above = [pick[0], pick[1], pick[2]+0.22]

    print(f'[PICK] Hovering above {obj}...')
    smooth_move(ik(above), steps=70, carrying=False)

    print(f'[PICK] Descending to {obj}...')
    # carrying=True from the very first frame of descent
    smooth_move(ik(pick), steps=40, carrying=True, carry_obj=obj)
    held_obj = obj
    snap_to_ee(obj)          # hard-lock at moment of contact

    print(f'[PICK] Lifting {obj}...')
    smooth_move(ik(above), steps=45, carrying=True, carry_obj=obj)

    print(f'[PICK] Returning home with {obj}...')
    smooth_move(home_arm_jpos, steps=65, carrying=True, carry_obj=obj)

    print(f'[PICK] Holding {obj}. Press Drop on Table.')


def do_drop(slot):
    """
    Drop sequence:
      1. Move over bed slot (carrying)
      2. Lower onto bed     (carrying)
      3. Release object
      4. Lift clear
      5. Return home
    """
    global held_obj

    obj        = held_obj
    drop_above = [slot[0], slot[1], slot[2]+0.22]

    print(f'[DROP] Moving to bed...')
    smooth_move(ik(drop_above), steps=65, carrying=True, carry_obj=obj)

    print(f'[DROP] Lowering onto bed...')
    smooth_move(ik(slot), steps=40, carrying=True, carry_obj=obj)

    print(f'[DROP] Releasing {obj}.')
    place_obj(obj, slot)     # snap to final resting place
    held_obj = ''

    print(f'[DROP] Lifting clear...')
    smooth_move(ik(drop_above), steps=40, carrying=False)

    print(f'[DROP] Returning home...')
    smooth_move(home_arm_jpos, steps=60, carrying=False)

    print(f'[DROP] Done.')


def do_home():
    global held_obj
    carrying = (held_obj != '')
    print('[HOME] Returning...')
    smooth_move(home_arm_jpos, steps=60,
                carrying=carrying, carry_obj=held_obj)
    print('[HOME] Done.')


def do_reset():
    global held_obj, drop_idx
    held_obj = ''
    drop_idx = 0
    for name, pos in ORIGIN.items():
        place_obj(name, pos)
    smooth_move(home_arm_jpos, steps=40, carrying=False)
    print('[RESET] All objects returned to tray.')


# ════════════════════════════════════════════════════════════
#  MAIN LOOP  — same structure as reference while True
# ════════════════════════════════════════════════════════════
print('=== SURGICAL ROBOT READY ===')
print('Press Pick Scalpel / Pick Forceps / Pick Suture to begin.')

while True:

    # Read all button values first
    vps = bb.readDebugParameter('Pick Scalpel')
    vpf = bb.readDebugParameter('Pick Forceps')
    vpu = bb.readDebugParameter('Pick Suture')
    vd  = bb.readDebugParameter('Drop on Table')
    vh  = bb.readDebugParameter('Return Home')
    vr  = bb.readDebugParameter('Reset All')

    # ── Pick buttons ──────────────────────────────────────
    if vps > old_vals['Pick Scalpel']:
        old_vals['Pick Scalpel'] = vps
        if held_obj:
            place_obj(held_obj, ORIGIN[held_obj])
            held_obj = ''
        do_pick('scalpel')           # BLOCKS until complete

    elif vpf > old_vals['Pick Forceps']:
        old_vals['Pick Forceps'] = vpf
        if held_obj:
            place_obj(held_obj, ORIGIN[held_obj])
            held_obj = ''
        do_pick('forceps')

    elif vpu > old_vals['Pick Suture']:
        old_vals['Pick Suture'] = vpu
        if held_obj:
            place_obj(held_obj, ORIGIN[held_obj])
            held_obj = ''
        do_pick('suture')

    # ── Drop ──────────────────────────────────────────────
    elif vd > old_vals['Drop on Table']:
        old_vals['Drop on Table'] = vd
        if held_obj:
            slot = BED_DROPS[drop_idx % len(BED_DROPS)]
            drop_idx += 1
            do_drop(slot)            # BLOCKS until complete

    # ── Return Home ───────────────────────────────────────
    elif vh > old_vals['Return Home']:
        old_vals['Return Home'] = vh
        do_home()

    # ── Reset All ─────────────────────────────────────────
    elif vr > old_vals['Reset All']:
        old_vals['Reset All'] = vr
        do_reset()

    # ── Idle: sliders control the arm ─────────────────────
    else:
        if not held_obj:
            # No object held — let sliders drive freely
            for i, jname in enumerate(JOINT_NAMES):
                jp = bb.readDebugParameter(jname)
                bb.setJointMotorControl(robot, i, targetPosition=jp)
                cur_j[i] = jp
        else:
            # Holding — keep object welded to EE
            snap_to_ee(held_obj)

    time.sleep(0.05)