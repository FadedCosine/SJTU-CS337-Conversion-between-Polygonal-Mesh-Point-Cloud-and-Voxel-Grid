# Greedy Projection Triangulation
Given a obj model, we first obtain all the vertices and normals information and divide them to normal.txt and point.txt.

Then using the original .obj file, normal.txt and point.txt as input, run the GreedyTrangulation.py to get reconstruction faces.txt.

Then we can integrate the face.txt with  normal.txt and point.txt to get a reconstructed mesh.

The parameters see parse_args() for details.
