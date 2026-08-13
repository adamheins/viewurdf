from xml.dom.minidom import parseString
import numpy as np
from scipy.spatial.transform import Rotation, RigidTransform


class Joint:
    def __init__(self, name, type_, axis, origin, parent_link_name, child_link_name):
        self.name = name
        self.type_ = type_
        self.axis = axis
        self.origin = origin
        self.parent = parent_link_name
        self.child = child_link_name

    @classmethod
    def from_element(cls, element):
        name = element.getAttribute("name")
        type_ = element.getAttribute("type")

        origin = element.getElementsByTagName("origin")[0]
        xyz = np.array(origin.getAttribute("xyz").split(" ")).astype(float)
        rpy = np.array(origin.getAttribute("rpy").split(" ")).astype(float)
        pose = RigidTransform.from_components(xyz, Rotation.from_euler("xyz", rpy))

        axes = element.getElementsByTagName("axis")
        if len(axes) > 0:
            axis = axes[0].getAttribute("xyz")
            axis = np.array(axis.split(" ")).astype(float)
            axis /= np.linalg.norm(axis)
        else:
            axis = np.array([1, 0, 0])

        parent_link_name = element.getElementsByTagName("parent")[0].getAttribute(
            "link"
        )
        child_link_name = element.getElementsByTagName("child")[0].getAttribute("link")

        return cls(
            name=name,
            type_=type_,
            axis=axis,
            origin=pose,
            parent_link_name=parent_link_name,
            child_link_name=child_link_name,
        )

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
    def __init__(self, origin=None, material=None):
        if origin is None:
            origin = RigidTransform.identity()
        self.origin = origin
        self.material = material

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

    def material_element(self, root):
        material = root.createElement("material")
        material.setAttribute("name", self.material)
        return material


class Cylinder(Geom):
    def __init__(self, length, radius, **kwargs):
        super().__init__(**kwargs)
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
    def __init__(self, radius, **kwargs):
        super().__init__(**kwargs)
        self.radius = radius

    def geometry_element(self, root):
        geometry = root.createElement("geometry")
        shape = root.createElement("sphere")
        shape.setAttribute("radius", str(self.radius))
        geometry.appendChild(shape)
        return geometry


class URDFSkeleton:
    def __init__(self, dom, link_radius=0.005, joint_radius=0.01):
        self.dom = dom
        self.links = {}
        self.joints = {}

        for joint in self.dom.getElementsByTagName("joint"):
            joint = Joint.from_element(joint)
            self.joints[joint.name] = joint
            self.links[joint.child] = Link(name=joint.child, parent=joint.name)

        for joint in self.joints.values():
            # base link is not picked up by the logic above
            if joint.parent not in self.links:
                continue
            self.links[joint.parent].children.append(joint.name)

        self._skeletonize(link_radius, joint_radius)

    @classmethod
    def from_string(cls, s, **kwargs):
        dom = parseString(s)
        return cls(dom)

    @classmethod
    def from_file(cls, path, **kwargs):
        with open(path) as f:
            s = f.read()
        return cls.from_string(s)

    def to_string(self):
        return self.dom.toxml()

    def _create_geometry(self, parent, container_name, geoms):
        for geom in geoms:
            container = self.dom.createElement(container_name)
            container.appendChild(geom.origin_element(root=self.dom))
            container.appendChild(geom.geometry_element(root=self.dom))
            if geom.material is not None:
                container.appendChild(geom.material_element(root=self.dom))
            parent.appendChild(container)

    def _create_material(self, name, rgba):
        robot = self.dom.getElementsByTagName("robot")[0]
        material = self.dom.createElement("material")
        material.setAttribute("name", name)
        color = self.dom.createElement("color")
        color.setAttribute("rgba", rgba)
        material.appendChild(color)
        robot.appendChild(material)

    def _skeletonize(self, link_radius, joint_radius):
        self._create_material(name="red", rgba="1.0 0.0 0.0 1.0")
        self._create_material(name="darkgrey", rgba="0.25 0.25 0.25 1.0")

        # TODO: need to actually make multiple links, which means additional
        # joints as well
        z = np.array([0, 0, 1])
        for link in self.links.values():

            # every link puts this at their parent joint origin

            # TODO need to look at parent joint
            parent = self.joints[link.parent]
            if parent.type_ == "revolute":
                R = Rotation.align_vectors(parent.axis, z)[0]
                T = RigidTransform.from_rotation(rotation=R)
                joint_geom = Cylinder(
                    radius=joint_radius, length=0.03, origin=T, material="red"
                )
            else:
                joint_geom = Ball(radius=joint_radius, material="darkgrey")
            link.geoms = [joint_geom]

            # add connections to all child joints
            for child in link.children:
                Δr = self.joints[child].origin.translation
                d = np.linalg.norm(Δr)

                # don't add geometry to length-zero links
                if np.isclose(d, 0):
                    continue

                R = Rotation.align_vectors(Δr, z)[0]
                T = RigidTransform.from_components(translation=0.5 * Δr, rotation=R)
                link.geoms.append(
                    Cylinder(
                        length=d, radius=link_radius, origin=T, material="darkgrey"
                    )
                )

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
            self._create_geometry(link, "visual", self.links[name].geoms)
