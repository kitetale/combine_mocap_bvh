import re
import numpy as np

from Animation import Animation
from Quaternions import Quaternions

channelmap = {
    'Xrotation': 'x',
    'Yrotation': 'y',
    'Zrotation': 'z'
}

channelmap_inv = {
    'x': 'Xrotation',
    'y': 'Yrotation',
    'z': 'Zrotation',
}

ordermap = {
    'x': 0,
    'y': 1,
    'z': 2,
}


def load(filename, start=None, end=None, order=None, world=False):
    """
    Reads a BVH file and constructs an animation
    
    Parameters
    ----------
    filename: str
        File to be opened
        
    start : int
        Optional Starting Frame
        
    end : int
        Optional Ending Frame
    
    order : str
        Optional Specifier for joint order.
        Given as string E.G 'xyz', 'zxy'
        
    world : bool
        If set to true euler angles are applied
        together in world space rather than local
        space

    Returns
    -------
    
    (animation, joint_names, frametime)
        Tuple of loaded animation and joint names
    """

    f = open(filename, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = Quaternions.id(0)
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    for line in f:

        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            jnum = len(parents)
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()
    rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
    return Animation(rotations, positions, orients, offsets, parents), names, frametime, order

#====================== REMOVE TOP DATA ========================================
def loadMinusTop(filename, start=None, end=None, order=None, world=False):
    """
    Reads a BVH file and constructs an animation
    
    Parameters
    ----------
    filename: str
        File to be opened
        
    start : int
        Optional Starting Frame
        
    end : int
        Optional Ending Frame
    
    order : str
        Optional Specifier for joint order.
        Given as string E.G 'xyz', 'zxy'
        
    world : bool
        If set to true euler angles are applied
        together in world space rather than local
        space

    Returns
    -------
    
    (animation, joint_names, frametime)
        Tuple of loaded animation and joint names
    """

    f = open(filename, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = Quaternions.id(0)
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    for line in f:

        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
                print(offmatch.groups())
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            jnum = len(parents)
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                print ("reshaping array of size %d into (%d, 3)\n" % (len(data_block[3:]), N))
                rotations[fi, :] = data_block[3:].reshape(N, 3)
                rotations[fi, :57] = np.zeros((1, 57, 3))
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()
    rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
    return Animation(rotations, positions, orients, offsets, parents), names, frametime, order

#======================= REMOVE BOTTOM DATA ====================================
def loadMinusBottom(filename, start=None, end=None, order=None, world=False):
    """
    Reads a BVH file and constructs an animation
    
    Parameters
    ----------
    filename: str
        File to be opened
        
    start : int
        Optional Starting Frame
        
    end : int
        Optional Ending Frame
    
    order : str
        Optional Specifier for joint order.
        Given as string E.G 'xyz', 'zxy'
        
    world : bool
        If set to true euler angles are applied
        together in world space rather than local
        space

    Returns
    -------
    
    (animation, joint_names, frametime)
        Tuple of loaded animation and joint names
    """

    f = open(filename, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = Quaternions.id(0)
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    for line in f:

        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
                print(offmatch.groups())
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            jnum = len(parents)
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                print ("reshaping array of size %d into (%d, 3)\n" % (len(data_block[3:]), N))
                rotations[fi, :] = data_block[3:].reshape(N, 3)
                rotations[fi, 57:] = np.zeros((1, 10, 3))
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()
    rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
    return Animation(rotations, positions, orients, offsets, parents), names, frametime, order


#=================== LOAD ONLY SPINE AND UP DATA ===============================
def loadtop(filename, start=None, end=None, order=None, world=False):
    """
    Reads a BVH file and constructs an animation
    
    Parameters
    ----------
    filename: str
        File to be opened
        
    start : int
        Optional Starting Frame
        
    end : int
        Optional Ending Frame
    
    order : str
        Optional Specifier for joint order.
        Given as string E.G 'xyz', 'zxy'
        
    world : bool
        If set to true euler angles are applied
        together in world space rather than local
        space

    Returns
    -------
    
    (animation, joint_names, frametime)
        Tuple of loaded animation and joint names
    """

    f = open(filename, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = Quaternions.id(0)
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    top = True

    for line in f:

        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: 
            continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue
        

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if (not end_site):
                if top:
                    offsets[active] = np.array([list(map(float, offmatch.groups()))])
                else:
                    offsets[active] = np.array([list(map(float, ('0.000000', '0.000000', '0.000000')))])
                
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            if (jmatch.group(1)=="Spine"):
                top = True
            if (jmatch.group(1)=="LeftUpLeg" or jmatch.group(1)=="RightUpLeg"):
                top = False
            # if top:
            print (jmatch.group(1)+"\n")
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue


        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            jnum = len(parents)
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()
    rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
    return Animation(rotations, positions, orients, offsets, parents), names, frametime, order

#========================== LOAD ONLY LEGS DATA ================================
def loadbottom(filename, start=None, end=None, order=None, world=False):
    """
    Reads a BVH file and constructs an animation
    
    Parameters
    ----------
    filename: str
        File to be opened
        
    start : int
        Optional Starting Frame
        
    end : int
        Optional Ending Frame
    
    order : str
        Optional Specifier for joint order.
        Given as string E.G 'xyz', 'zxy'
        
    world : bool
        If set to true euler angles are applied
        together in world space rather than local
        space

    Returns
    -------
    
    (animation, joint_names, frametime)
        Tuple of loaded animation and joint names
    """

    f = open(filename, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = Quaternions.id(0)
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    bottom = True

    for line in f:

        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: 
            continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue
        

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if (not end_site):
                if bottom:
                    offsets[active] = np.array([list(map(float, offmatch.groups()))])
                else:
                    offsets[active] = np.array([list(map(float, ('0.000000', '0.000000', '0.000000')))])
                
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            if (jmatch.group(1)=="Spine"):
                bottom = False
            if (jmatch.group(1)=="LeftUpLeg" or jmatch.group(1)=="RightUpLeg"):
                bottom = True
            # if top:
            print (jmatch.group(1)+"\n")
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue


        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            jnum = len(parents)
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
                rotations[fi, :57] = np.zeros((1, 57, 3))
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()
    rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
    return Animation(rotations, positions, orients, offsets, parents), names, frametime, order



#=========================== MERGE TWO ANIM DATA ===============================
def combine(anim1, anim2):
    f1 = anim1.rotations.qs.shape[0] # num of frames in anim1
    f2 = anim2.rotations.qs.shape[0] # num of frames in anim2
    fnum = f1 if f1>f2 else f2 # set greater number as frame num
    fmin = f1 if f1<f2 else f1 # the other fnum
    animMax = anim1 if f1<f2 else anim2
    longer = f1>f2
    jnum = anim1.rotations.qs.shape[1] # num of joints 
    assert(jnum == anim2.rotations.qs.shape[1]) #double checking num of joints r the same

    # init combined pos and rot
    # positions = offsets[np.newaxis].repeat(fnum, axis=0)
    positions = animMax.positions
    rotations = np.zeros((fnum, jnum, 4))

    for fi in range (fnum):
        if fi < fmin:
            rotations[fi, :57] = anim1.rotations[fi, :57]
            rotations[fi, 57:] = anim2.rotations[fi, 10]
        else:
            if longer: # anim1 got more frames
                rotations[fi, :57] = anim1.rotations[fi, :57]
                rotations[fi, 57:] = np.zeros((1, 10, 4))
            else: # anim2 got more frames
                rotations[fi, :57] = np.zeros((1, 57, 4))
                rotations[fi, 57:] = anim2.rotations[fi, 10]


    return Animation(rotations, positions, anim1.orients, anim1.offsets, anim1.parents)


def loadTwo(filename1, filename2, start=None, end=None, order=None, world=False):
    """
    Reads a BVH file and constructs an animation
    
    Parameters
    ----------
    filename1: str
        File to be opened (top)
    filename2: str
        File to be opened (bottom)
        
    start : int
        Optional Starting Frame
        
    end : int
        Optional Ending Frame
    
    order : str
        Optional Specifier for joint order.
        Given as string E.G 'xyz', 'zxy'
        
    world : bool
        If set to true euler angles are applied
        together in world space rather than local
        space

    Returns
    -------
    
    (animation, joint_names, frametime)
        Tuple of loaded animation and joint names
    """

    f1 = open(filename1, "r")
    f2 = open(filename2, "r")

    i = 0
    active = -1
    end_site = False

    names = []
    orients = Quaternions.id(0)
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    fmax = 0
    fmin = 0

    for line in f1:

        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients.qs = np.append(orients.qs, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "End Site" in line:
            end_site = True
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
                fmax = fnum
                fmin = fnum
            jnum = len(parents)
            positions1 = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations1 = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions1[fi, 0:1] = data_block[0:3]
                rotations1[fi, :] = data_block[3:].reshape(N, 3)
                rotations1[fi, 57:] = np.zeros((1, 10, 3)) # remove bottom data
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)

            i += 1

    moreFrames = True
    i = 0

    for line in f2:
        # skip all until data block
        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            continue

        if "{" in line: continue

        if "}" in line:
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            continue

        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            continue

        if "End Site" in line:
            continue

        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
                if fnum>fmax: 
                    fmax = fnum
                    moreFrames = True
                else: 
                    fmin = fnum
                    moreFrames = False
            positions2 = offsets[np.newaxis].repeat(fnum, axis=0)
            positions = offsets[np.newaxis].repeat(fmax, axis=0)
            rotations2 = np.zeros((fnum, len(orients), 3))
            rotations = np.zeros((fmax, len(orients), 3))
            continue

        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue
        
        # data block
        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = 67
            fi = i - start if start else i
            if (fi>=fmin):
                if (moreFrames): #file 2 has more frames
                    if channels == 3:
                        positions[fi, 0:1] = data_block[0:3]
                        rotations2[fi, :] = data_block[3:].reshape(N, 3)
                        # rotations[fi, :57] = rotations1[fi, :57]
                        rotations[fi, :] = rotations2[fi, :]
                        rotations[fi, :57] = np.zeros((1, 57, 3))

                    
                    else:
                        raise Exception("Too many channels! %i" % channels)
                else: # file 1 has more frames
                    if channels == 3:
                        positions[fi, 0:1] = np.zeros((1, 1, 3))
                        rotations[fi, :] = rotations1[fi, :]
                        rotations[f1, 57:] = np.zeros((1, 10, 3))
                   
                    else:
                        raise Exception("Too many channels! %i" % channels)
                
            else:        
                if channels == 3:
                    # if moreFrames:
                    positions[fi, 0:1] = data_block[0:3]
                    # else:
                    #     positions[fi, 0:1] = positions1[fi, 0:1]
                    rotations2[fi, :] = data_block[3:].reshape(N, 3)
                    rotations[fi,:] = rotations1[fi, :]
                    rotations[fi, 57:] = rotations2[fi, 57:]
               
                else:
                    raise Exception("Too many channels! %i" % channels)

            i += 1



    f1.close()
    f2.close()
    rotations = Quaternions.from_euler(np.radians(rotations), order=order, world=world)
    return Animation(rotations, positions, orients, offsets, parents), names, frametime, order


#=================================== SAVE ======================================


def save(filename, anim, names=None, frametime=1.0 / 24.0, order='zyx', positions=False):
    """
    Saves an Animation to file as BVH
    
    Parameters
    ----------
    filename: str
        File to be saved to
        
    anim : Animation
        Animation to save
        
    names : [str]
        List of joint names
    
    order : str
        Optional Specifier for joint order.
        Given as string E.G 'xyz', 'zxy'
    
    frametime : float
        Optional Animation Frame time
        
    positions : bool
        Optional specfier to save bone
        positions for each frame
    """

    if names is None:
        names = ["joint_" + str(i) for i in range(len(anim.parents))]

    with open(filename, 'w') as f:

        t = ""
        f.write("%sHIERARCHY\n" % t)
        f.write("%sROOT %s\n" % (t, names[0]))
        f.write("%s{\n" % t)
        t += '\t'

        f.write("%sOFFSET %f %f %f\n" % (t, anim.offsets[0, 0], anim.offsets[0, 1], anim.offsets[0, 2]))
        f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" %
                (t, channelmap_inv[order[0]], channelmap_inv[order[1]], channelmap_inv[order[2]]))

        for i in range(anim.shape[1]):
            if anim.parents[i] == 0:
                t = save_joint(f, anim, names, t, i, order=order, positions=positions)

        t = t[:-1]
        f.write("%s}\n" % t)

        f.write("MOTION\n")
        f.write("Frames: %i\n" % anim.shape[0])
        f.write("Frame Time: %f\n" % frametime)

        rots = np.degrees(anim.rotations.euler(order=order[::-1]))
        poss = anim.positions

        for i in range(anim.shape[0]):
            for j in range(anim.shape[1]):

                if positions or j == 0:

                    f.write("%f %f %f %f %f %f " % (
                        poss[i, j, 0], poss[i, j, 1], poss[i, j, 2],
                        rots[i, j, ordermap[order[0]]], rots[i, j, ordermap[order[1]]], rots[i, j, ordermap[order[2]]]))

                else:

                    f.write("%f %f %f " % (
                        rots[i, j, ordermap[order[0]]], rots[i, j, ordermap[order[1]]], rots[i, j, ordermap[order[2]]]))

            f.write("\n")


def save_joint(f, anim, names, t, i, order='zyx', positions=False):
    f.write("%sJOINT %s\n" % (t, names[i]))
    f.write("%s{\n" % t)
    t += '\t'

    f.write("%sOFFSET %f %f %f\n" % (t, anim.offsets[i, 0], anim.offsets[i, 1], anim.offsets[i, 2]))

    if positions:
        f.write("%sCHANNELS 6 Xposition Yposition Zposition %s %s %s \n" % (t,
                                                                            channelmap_inv[order[0]],
                                                                            channelmap_inv[order[1]],
                                                                            channelmap_inv[order[2]]))
    else:
        f.write("%sCHANNELS 3 %s %s %s\n" % (t,
                                             channelmap_inv[order[0]], channelmap_inv[order[1]],
                                             channelmap_inv[order[2]]))

    end_site = True

    for j in range(anim.shape[1]):
        if anim.parents[j] == i:
            t = save_joint(f, anim, names, t, j, order=order, positions=positions)
            end_site = False

    if end_site:
        f.write("%sEnd Site\n" % t)
        f.write("%s{\n" % t)
        t += '\t'
        f.write("%sOFFSET %f %f %f\n" % (t, 0.0, 0.0, 0.0))
        t = t[:-1]
        f.write("%s}\n" % t)

    t = t[:-1]
    f.write("%s}\n" % t)

    return t
