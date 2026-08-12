import urdf_skeleton as skel
import sys

def main():
    urdf = skel.URDF.from_file(sys.argv[1])
    # print(urdf.links)
    urdf.skeletonize()
    print(urdf.to_string())


if __name__ == "__main__":
    main()
