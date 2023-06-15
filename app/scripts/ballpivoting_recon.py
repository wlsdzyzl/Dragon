import pymeshlab
import sys
def load_and_recon(input_path, output_path):
    # lines needed to run this specific example
    # create a new MeshSet
    ms = pymeshlab.MeshSet()

    # load a new mesh in the MeshSet, and sets it as current mesh
    # the path of the mesh can be absolute or relative
    ms.load_new_mesh(input_path)


    # apply the filter script contained in the MeshSet
    # note - the filter script used in this example does not require an input mesh,
    #        therefore it can be applied also in an empty MeshSet
    ms.generate_surface_reconstruction_ball_pivoting()

    ms.save_current_mesh(output_path)
if __name__ == "__main__":
    load_and_recon(sys.argv[1], sys.argv[2])