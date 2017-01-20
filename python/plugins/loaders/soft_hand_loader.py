__author__ = 'Alessio Rocchi'

import argparse
from lxml import etree

class SoftHandLoader(object):
    def __init__(self,filename):
        self.handParameters = dict()
        self.jointToLink = dict()

        self.urdf = etree.fromstring(file(filename).read())

        for transmission_el in self.urdf.iter('transmission'):
            for transmission_type_el in transmission_el.iter('type'):
                if isinstance(transmission_type_el.tag, basestring):
                    if transmission_type_el.text == 'transmission_interface/AdaptiveSynergyTransmission':
                        self.handParameters = self.parseTransmission(transmission_el)

        self.jointToLink = self.parseJointToLink()

    def parseTransmission(self, transmission_el):
        handParams = dict()
        for joint_el in transmission_el.iter('joint'):
            if isinstance(joint_el.tag, basestring):
                joint_name = joint_el.get('name')
                _,_,finger,joint,_ = joint_name.split('_')
                for R_el in joint_el.iter('mechanicalReduction'):
                    if not handParams.has_key(finger):
                        handParams[finger] = dict()
                    handParams[finger][joint] = {'r':float(R_el.text)}
                for E_el in joint_el.iter('mechanicalElasticity'):
                    handParams[finger][joint]['e']=float(E_el.text)
        return handParams

    def parseJointToLink(self):
        jointToLink = dict()
        for joint_el in self.urdf.iter('joint'):
            if isinstance(joint_el.tag, basestring):
                if 'type' in joint_el.keys() and joint_el.get('type') == 'revolute':
                    joint_name = joint_el.get('name')
                    is_mimic = (joint_name.split('_')[-1]=='mimic')
                    if is_mimic:
                        continue
                    jointToLink[joint_name] = self.parseJointChildLink(joint_name)
        return jointToLink


    def parseJointChildLink(self,joint_name):
        for joint_el in self.urdf.iter('joint'):
            if isinstance(joint_el.tag, basestring):
                if 'name' in joint_el.keys() and joint_el.get('name') == joint_name:
                    if 'type' in joint_el.keys() and joint_el.get('type') == 'revolute':
                        for child_link_el in joint_el.iter('child'):
                            if isinstance(joint_el.tag, basestring):
                                if 'link' in child_link_el.keys():
                                    link_name = child_link_el.get('link')
                                    _,_,_,fake,_ = link_name.split('_')
                                    is_fake = (fake == 'fake')
                                    if not is_fake:
                                        return link_name
                                    else:
                                        childLinkChildJoint = self.parseChildWithParentLink(link_name)
                                        return self.parseJointChildLink(childLinkChildJoint)
        raise Exception('could not find child link for joint %s'%joint_name)

    def parseChildWithParentLink(self,link_name):
        for joint_el in self.urdf.iter('joint'):
            if isinstance(joint_el.tag, basestring):
                if 'type' in joint_el.keys() and joint_el.get('type') == 'revolute':
                    for child_link_el in joint_el.iter('parent'):
                        if isinstance(joint_el.tag, basestring):
                            if 'link' in child_link_el.keys():
                                if child_link_el.get('link') == link_name:
                                    return joint_el.get('name')
        raise Exception('could not joint with parent link %s'%link_name)


    def jointToPhalanx(self, finger, joint_position):
        _,_,_,phalanx,_= self.jointToLink['soft_hand_%s_%s_joint'%(finger,joint_position)].split('_')
        return phalanx

    def phalanxToJoint(self, finger, phalanx):
        for key,val in self.jointToLink.iteritems():
            if val == 'soft_hand_%s_%s_link'%(finger,phalanx):
                joint_name = key.split('_')[3]
                return joint_name

        raise Exception('could not find parent joint for link soft_hand_%s_%s_link'%(finger,phalanx))


if __name__ == '__main__':

    parser = argparse.ArgumentParser(usage='SoftHandLoader soft_hand_urdf_file\nLoad an URDF file and gets transmission information out of it')
    parser.add_argument('urdf_file', type=argparse.FileType('r'), nargs='?',
                        default=None, help='URDF file. Use - for stdin')
    args = parser.parse_args()
    # Extract robot name and directory

    if args.urdf_file is None:
        print("Error! no urdf_file provided")
        exit()
    else:
        loader = SoftHandLoader(args.urdf_file)

    print(loader.handParams)
