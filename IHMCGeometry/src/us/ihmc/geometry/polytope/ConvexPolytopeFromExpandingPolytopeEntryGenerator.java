package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point3d;

public class ConvexPolytopeFromExpandingPolytopeEntryGenerator
{

   public ConvexPolytope generateConvexPolytope(ExpandingPolytopeEntry expandingPolytope)
   {
      ConvexPolytope convexPolytope = new ConvexPolytope();

      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(triangles);

      HashMap<Point3d, PolytopeVertex> pointsToPolytopeVertices = new HashMap<>();

      for (ExpandingPolytopeEntry triangle : triangles)
      {
         for (int i = 0; i < 3; i++)
         {
            Point3d vertex = triangle.getVertex(i);

            if (!pointsToPolytopeVertices.containsKey(vertex))
            {
               PolytopeVertex polytopeVertex = convexPolytope.addVertex(vertex);
               pointsToPolytopeVertices.put(vertex, polytopeVertex);
            }
         }
      }

      for (ExpandingPolytopeEntry triangle : triangles)
      {
         for (int i = 0; i < 3; i++)
         {
            Point3d vertex = triangle.getVertex(i);
            Point3d nextVertex = triangle.getVertex((i + 1) % 3);

            PolytopeVertex polytopeVertex = pointsToPolytopeVertices.get(vertex);
            PolytopeVertex nextPolytopeVertex = pointsToPolytopeVertices.get(nextVertex);

            convexPolytope.addEdge(polytopeVertex, nextPolytopeVertex);
         }
      }

      return convexPolytope;
   }
}