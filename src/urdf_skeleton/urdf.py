from xml.dom.minidom import parseString
import numpy as np
from scipy.spatial.transform import Rotation, RigidTransform


class Joint:
    def __init__(self, name, xyz, rpy, parent, child):
        self.name = name
        self.pose = RigidTransform.from_components(xyz, Rotation.from_euler("xyz", rpy))
        self.parent = parent
        self.child = child

    def __repr__(self):
        return f"Joint(name={self.name}, parent={self.parent}, child={self.child}, pose={self.pose})"


class Link:
    def __init__(self, name, parent=None):
        self.name = name
        self.parent = parent
        self.children = []

    def __repr__(self):
        return f"Link(name={self.name}, parent={self.parent}, children={self.children})"


class Geom:
    def __init__(self, origin=None):
        if origin is None:
            origin = RigidTransform.identity()
        self.origin = origin

    @property
    def rpy(self):
        return " ".join(
            self.origin.rotation.as_euler("xyz", suppress_warnings=True).astype(str)
        )

    @property
    def xyz(self):
        return " ".join(self.origin.translation.astype(str))

    def origin_element(self, root):
        origin = root.createElement("origin")
        origin.setAttribute("xyz", self.xyz)
        origin.setAttribute("rpy", self.rpy)
        return origin


class Cylinder(Geom):
    def __init__(self, length, radius, origin=None):
        super().__init__(origin)
        self.length = length
        self.radius = radius

    def geometry_element(self, root):
        geometry = root.createElement("geometry")
        shape = root.createElement("cylinder")
        shape.setAttribute("radius", str(self.radius))
        shape.setAttribute("length", str(self.length))
        geometry.appendChild(shape)
        return geometry


class Ball(Geom):
    def __init__(self, radius, origin=None):
        super().__init__(origin)
        self.radius = radius

    def geometry_element(self, root):
        geometry = root.createElement("geometry")
        shape = root.createElement("sphere")
        shape.setAttribute("radius", str(self.radius))
        geometry.appendChild(shape)
        return geometry


class URDF:
    def __init__(self, dom):
        self.dom = dom
        self.links = {}
        self.joints = {}

        for joint in self.dom.getElementsByTagName("joint"):
            name = joint.getAttribute("name")
            origin = joint.getElementsByTagName("origin")[0]
            xyz = np.array(origin.getAttribute("xyz").split(" ")).astype(float)
            rpy = np.array(origin.getAttribute("rpy").split(" ")).astype(float)

            parent_link_name = joint.getElementsByTagName("parent")[0].getAttribute(
                "link"
            )
            child_link_name = joint.getElementsByTagName("child")[0].getAttribute(
                "link"
            )

            self.joints[name] = Joint(
                name=name,
                xyz=xyz,
                rpy=rpy,
                parent=parent_link_name,
                child=child_link_name,
            )
            self.links[child_link_name] = Link(name=child_link_name, parent=name)

        for joint in self.joints.values():
            # TODO there will always be one where this doesn't work
            try:
                self.links[joint.parent].children.append(joint.name)
            except KeyError:
                pass

    @classmethod
    def from_string(cls, s):
        dom = parseString(s)
        return cls(dom)

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            s = f.read()
        return cls.from_string(s)

    def create_geometry(self, parent, container_name, geoms):
        for geom in geoms:
            container = self.dom.createElement(container_name)
            container.appendChild(geom.origin_element(root=self.dom))
            container.appendChild(geom.geometry_element(root=self.dom))
            parent.appendChild(container)

    def skeletonize(self, r=0.01):
        # for joint in self.joints.values():
        #     print(f"{joint.name}: {joint.pose.translation}")

        z = np.array([0, 0, 1])
        for link in self.links.values():
            link.geoms = [Ball(radius=0.05)]
            for child in link.children:
                Δr = self.joints[child].pose.translation
                d = np.linalg.norm(Δr)

                # don't add geometry to length-zero links
                if np.isclose(d, 0):
                    continue

                R = Rotation.align_vectors(z, Δr)[0]
                T = RigidTransform.from_components(translation=0.5 * Δr, rotation=R)
                link.geoms.append(Cylinder(length=d, radius=r, origin=T))

        # remove existing link child elements
        for link in self.dom.getElementsByTagName("link"):
            children = [n for n in link.childNodes if n.nodeType == n.ELEMENT_NODE]
            for child in children:
                link.removeChild(child)

        # build new visual and collision elements
        for link in self.dom.getElementsByTagName("link"):
            name = link.getAttribute("name")
            if name not in self.links:
                continue
            self.create_geometry(link, "visual", self.links[name].geoms)

    def to_string(self):
        # return self.dom.toprettyxml(indent="  ")
        return self.dom.toxml()
