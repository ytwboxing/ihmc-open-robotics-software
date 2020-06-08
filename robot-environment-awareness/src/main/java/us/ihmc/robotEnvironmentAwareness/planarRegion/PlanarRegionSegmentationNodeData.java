package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.*;
import java.util.stream.Stream;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.geometry.PointMean;
import us.ihmc.robotEnvironmentAwareness.geometry.VariancePredictor;
import us.ihmc.robotEnvironmentAwareness.geometry.VectorMean;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class PlanarRegionSegmentationNodeData implements Iterable<NormalOcTreeNode>
{
   private int id = PlanarRegion.NO_REGION_ID;

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
   private final VectorMean normal = new VectorMean();
   private final PointMean point = new PointMean();
   private final Vector3D standardDeviationPrincipalValues = new Vector3D();
   private final Vector3D temporaryVector = new Vector3D();

   private final VariancePredictor zVariance = new VariancePredictor();
   private final List<NormalOcTreeNode> nodes = new ArrayList<>();
   private final Set<NormalOcTreeNode> nodeSet = new HashSet<>();
   private final BoundingBox3D boundingBox = new BoundingBox3D();

   public PlanarRegionSegmentationNodeData(int id)
   {
      this.id = id;
   }

   public PlanarRegionSegmentationNodeData(int id, Collection<NormalOcTreeNode> nodes)
   {
      this(id);
      addNodes(nodes);
   }

   public boolean addNode(NormalOcTreeNode node)
   {
      boolean isRegionModified = nodeSet.add(node);
      if (isRegionModified)
      {
         updateNormalAndOriginOnly(node);
         nodes.add(node);
         updateBoundingBoxWithNewNode(node);
      }
      return isRegionModified;
   }

   public boolean addNodes(Collection<NormalOcTreeNode> nodes)
   {
      return nodes.stream().anyMatch(this::addNode);
   }

   public boolean addNodesFromOtherRegion(PlanarRegionSegmentationNodeData other)
   {
      return other.nodeStream().anyMatch(this::addNode);
   }

   public boolean contains(NormalOcTreeNode node)
   {
      return nodeSet.contains(node);
   }

   public void removeNodesAndUpdate(Collection<NormalOcTreeNode> nodesToRemove)
   {
      boolean containsAtLeastOne = nodesToRemove.parallelStream().anyMatch(nodeSet::contains);

      if (containsAtLeastOne)
      {
         nodes.removeAll(nodesToRemove);
         recomputeNormalAndOrigin();
         nodesToRemove.stream().filter(nodeSet::remove).forEach(this::updateBoundingBoxAfterRemovingNode);
      }
   }

   public void recomputeNormalAndOrigin()
   {
      pca.clear();
      // TODO weight the addition in the PCA by the number of points in the node
      nodes.forEach(node -> pca.addPoint(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ()));
      pca.compute();

      Point3D mean = new Point3D();
      pca.getMean(mean);

      point.clear();
      point.update(mean, getNumberOfNodes());

      Vector3D thirdVector = new Vector3D();
      pca.getThirdVector(thirdVector);
      pca.getStandardDeviation(standardDeviationPrincipalValues);

      if (thirdVector.dot(normal) < 0.0)
         thirdVector.negate();
      normal.clear();
      normal.update(thirdVector, getNumberOfNodes());
   }

   public double predictZVarianceIfAdded(NormalOcTreeNode node)
   {
      // TODO weight the addition in the variance by the number of points in the node.
      node.getNormal(temporaryVector);
      double zDirection = temporaryVector.getZ();
      if (getNumberOfNodes() >= 1 && temporaryVector.dot(normal) < 0.0)
         zDirection = -zDirection;

      return zVariance.predictIncrement(zDirection);
   }

   private void updateNormalAndOriginOnly(NormalOcTreeNode node)
   {
      node.getNormal(temporaryVector);
      // TODO Review and possibly improve dealing with normal flips.
      if (getNumberOfNodes() >= 1 && temporaryVector.dot(normal) < 0.0)
         temporaryVector.negate();
      int numberOfHits = (int) node.getNumberOfHits();
      normal.update(temporaryVector, numberOfHits);
      point.update(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ(), numberOfHits);
      zVariance.increment(temporaryVector.getZ());
   }

   public double distanceFromBoundingBox(NormalOcTreeNode node)
   {
      return EuclidCoreMissingTools.distanceToBoundingBox3D(node.getX(), node.getY(), node.getZ(), boundingBox);
   }

   public double distanceSquaredFromOtherRegionBoundingBox(PlanarRegionSegmentationNodeData other)
   {
      return EuclidCoreMissingTools.distanceSquaredBetweenTwoBoundingBox3Ds(boundingBox, other.boundingBox);
   }

   private void updateBoundingBoxWithNewNode(NormalOcTreeNode newNode)
   {
      boundingBox.updateToIncludePoint(newNode.getX(), newNode.getY(), newNode.getZ());
   }

   private void updateBoundingBoxAfterRemovingNode(NormalOcTreeNode removedNode)
   {
      Point3DBasics min = boundingBox.getMinPoint();
      Point3DBasics max = boundingBox.getMaxPoint();
      if (nodes.isEmpty())
      {
         min.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
         max.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }

      double nodeX = removedNode.getX();
      double nodeY = removedNode.getY();
      double nodeZ = removedNode.getZ();
      double epsilon = 1.0e-3;

      if (Math.abs(nodeX - min.getX()) < epsilon)
         min.setX(findMin((node1, node2) -> node1.getX() < node2.getX() ? -1 : 1).getX());
      else if (Math.abs(nodeX - max.getX()) < epsilon)
         max.setX(findMax((node1, node2) -> node1.getX() < node2.getX() ? -1 : 1).getX());

      if (Math.abs(nodeY - min.getY()) < epsilon)
         min.setY(findMin((node1, node2) -> node1.getY() < node2.getY() ? -1 : 1).getY());
      else if (Math.abs(nodeY - max.getY()) < epsilon)
         max.setY(findMax((node1, node2) -> node1.getY() < node2.getY() ? -1 : 1).getY());

      if (Math.abs(nodeZ - min.getZ()) < epsilon)
         min.setZ(findMin((node1, node2) -> node1.getZ() < node2.getZ() ? -1 : 1).getZ());
      else if (Math.abs(nodeZ - max.getZ()) < epsilon)
         max.setZ(findMax((node1, node2) -> node1.getZ() < node2.getZ() ? -1 : 1).getZ());
   }

   private NormalOcTreeNode findMin(Comparator<NormalOcTreeNode> nodeComparator)
   {
      return nodes.parallelStream().min(nodeComparator).get();
   }

   private NormalOcTreeNode findMax(Comparator<NormalOcTreeNode> nodeComparator)
   {
      return nodes.parallelStream().max(nodeComparator).get();
   }

   public int getId()
   {
      return id;
   }

   public void getPoint(int index, Point3DBasics pointToPack)
   {
      nodes.get(index).getHitLocation(pointToPack);
   }

   public NormalOcTreeNode getNode(int index)
   {
      return nodes.get(index);
   }

   public double orthogonalDistance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, this.point, normal);
   }

   public double absoluteOrthogonalDistance(Point3DReadOnly point)
   {
      return Math.abs(orthogonalDistance(point));
   }

   public double orhtogonalDistance(NormalOcTreeNode node)
   {
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ(), point, normal);
   }

   public double absoluteOrthogonalDistance(NormalOcTreeNode node)
   {
      return Math.abs(orhtogonalDistance(node));
   }

   public double angle(Vector3DReadOnly normal)
   {
      return this.normal.angle(normal);
   }

   public double absoluteAngle(Vector3DReadOnly normal)
   {
      return Math.abs(angle(normal));
   }

   public double dot(Vector3DReadOnly normal)
   {
      return this.normal.dot(normal);
   }

   public double absoluteDot(Vector3DReadOnly normal)
   {
      return Math.abs(dot(normal));
   }

   public double dot(PlanarRegionSegmentationNodeData other)
   {
      return dot(other.normal);
   }

   public double absoluteDot(PlanarRegionSegmentationNodeData other)
   {
      return Math.abs(dot(other));
   }

   public double dotWithNodeNormal(NormalOcTreeNode node)
   {
      return normal.getX() * node.getNormalX() + normal.getY() * node.getNormalY() + normal.getZ() * node.getNormalZ();
   }

   public double absoluteDotWithNodeNormal(NormalOcTreeNode node)
   {
      return Math.abs(dotWithNodeNormal(node));
   }

   public Vector3D getNormal()
   {
      return normal;
   }

   public Point3D getOrigin()
   {
      return point;
   }

   public Vector3D getStandardDeviationPrincipalValues()
   {
      return standardDeviationPrincipalValues;
   }

   public boolean isEmpty()
   {
      return nodes.isEmpty();
   }

   public int getNumberOfNodes()
   {
      return nodes.size();
   }

   public List<NormalOcTreeNode> getNodes()
   {
      return nodes;
   }

   public Stream<NormalOcTreeNode> nodeStream()
   {
      return nodes.stream();
   }

   public Stream<NormalOcTreeNode> nodeParallelStream()
   {
      return nodes.parallelStream();
   }

   @Override
   public Iterator<NormalOcTreeNode> iterator()
   {
      return nodes.iterator();
   }

   @Override
   public String toString()
   {
      String ret = "Region ID: " + id;
      ret += ", origin: " + point + ", normal: " + normal;
      ret += ", size: " + getNumberOfNodes();
      return ret;
   }
}
