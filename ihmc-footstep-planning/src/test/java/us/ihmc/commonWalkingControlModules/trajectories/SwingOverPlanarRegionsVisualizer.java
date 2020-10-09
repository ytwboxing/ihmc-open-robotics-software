package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsCollisionType;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.*;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingOverPlanarRegionsVisualizer
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;

   private final YoFrameConvexPolygon2D yoFootPolygon;
   private final YoFrameVector3D planeNormal;
   private final YoFramePoint3D planeOrigin;
   private final YoFramePoseUsingYawPitchRoll solePose;
   private final YoFramePoint3D actualFootBoxPoint;
   private final YoFrameYawPitchRoll actualFootBoxOrientation;
   private final YoGraphicShape actualFootGraphicBox;
   private final YoFramePoint3D collisionBoxPoint;
   private final YoFrameYawPitchRoll collisionBoxOrientation;
   private final YoGraphicShape collisionGraphicBox;
   private final PoseReferenceFrame actualFootBoxReferenceFrame = new PoseReferenceFrame("footPose", WORLD);
   private final FramePose3D collisionBoxPose = new FramePose3D();
   private final YoFramePoint3D swingFootPoint;
   private final YoFramePoint3D firstWaypoint;
   private final YoFramePoint3D secondWaypoint;
   private final YoGraphicPolygon swingFoot;
   private final YoGraphicPolygon stanceFootGraphic;
   private final YoGraphicPolygon swingStartGraphic;
   private final YoGraphicPolygon swingEndGraphic;
   private final YoGraphicPosition firstWaypointGraphic;
   private final YoGraphicPosition secondWaypointGraphic;
   private final YoGraphicVector planeVector;
   private final Map<SwingOverPlanarRegionsCollisionType, YoGraphicPosition> intersectionMap;

   private final ConvexPolygon2DReadOnly footPolygon;
   private final SwingOverPlanarRegionsTrajectoryExpander trajectoryExpander;

   public SwingOverPlanarRegionsVisualizer(SimulationConstructionSet scs,
                                           YoRegistry registry,
                                           YoGraphicsListRegistry yoGraphicsListRegistry,
                                           ConvexPolygon2DReadOnly footPolygon,
                                           SwingOverPlanarRegionsTrajectoryExpander trajectoryExpander)
   {
      this.scs = scs;
      this.trajectoryExpander = trajectoryExpander;
      this.footPolygon = footPolygon;

      yoFootPolygon = new YoFrameConvexPolygon2D("footPolygon", ReferenceFrame.getWorldFrame(), 4, registry);
      yoFootPolygon.set(footPolygon);

      trajectoryExpander.attachVisualizer(this::update);

      double footHeight = trajectoryExpander.getFootHeight();

      solePose = new YoFramePoseUsingYawPitchRoll("SolePose", WORLD, registry);
      swingFootPoint = new YoFramePoint3D("SwingFootPoint", WORLD, registry);
      firstWaypoint = new YoFramePoint3D("FirstWaypointViz", WORLD, registry);
      secondWaypoint = new YoFramePoint3D("SecondWaypointViz", WORLD, registry);
      AppearanceDefinition bubble = YoAppearance.LightBlue();
      bubble.setTransparency(0.5);
      YoGraphicPosition trajectoryPosition = new YoGraphicPosition("TrajectoryPosition", swingFootPoint, 0.03, YoAppearance.Red());

      Graphics3DObject actualFootGraphicDefinition = new Graphics3DObject();
      FrameBox3D collisionBox = trajectoryExpander.getCollisionBox();
      AppearanceDefinition actualFootBoxAppearance = YoAppearance.CornflowerBlue();
      actualFootBoxAppearance.setTransparency(0.6);
      actualFootGraphicDefinition.addCube(collisionBox.getSizeX(), collisionBox.getSizeY(), collisionBox.getSizeZ(), false, actualFootBoxAppearance);
      actualFootBoxPoint = new YoFramePoint3D("ActualFootBoxPoint", WORLD, registry);
      actualFootBoxOrientation = new YoFrameYawPitchRoll("ActualFootBoxOrientation", WORLD, registry);
      actualFootGraphicBox = new YoGraphicShape("ActualFootGraphicBox", actualFootGraphicDefinition, actualFootBoxPoint, actualFootBoxOrientation, 1.0);
      AppearanceDefinition collisionBoxAppearance = YoAppearance.DarkRed();
      collisionBoxAppearance.setTransparency(0.8);
      double extraSize = trajectoryExpander.getMinimumClearance() * 2.0;
      Graphics3DObject collisionBoxGraphicDefinition = new Graphics3DObject();
      collisionBoxGraphicDefinition.addCube(collisionBox.getSizeX() + extraSize,
                                            collisionBox.getSizeY() + extraSize,
                                            collisionBox.getSizeZ() + extraSize,
                                            false,
                                            collisionBoxAppearance);
      collisionBoxPoint = new YoFramePoint3D("CollisionBoxPoint", WORLD, registry);
      collisionBoxOrientation = new YoFrameYawPitchRoll("CollisionBoxOrientation", WORLD, registry);
      collisionGraphicBox = new YoGraphicShape("CollisionGraphicBox", collisionBoxGraphicDefinition, collisionBoxPoint, collisionBoxOrientation, 1.0);
      swingFoot = new YoGraphicPolygon("SwingFoot", yoFootPolygon, solePose.getPosition(), solePose.getYawPitchRoll(), 1.0, footHeight, YoAppearance.Green());
      stanceFootGraphic = new YoGraphicPolygon("StanceFootGraphic", footPolygon.getNumberOfVertices(), registry, true, 1.0, YoAppearance.Blue());
      swingStartGraphic = new YoGraphicPolygon("SwingStartGraphic", footPolygon.getNumberOfVertices(), registry, true, 1.0, YoAppearance.Green());
      swingEndGraphic = new YoGraphicPolygon("SwingEndGraphic", footPolygon.getNumberOfVertices(), registry, true, 1.0, YoAppearance.Yellow());
      firstWaypointGraphic = new YoGraphicPosition("FirstWaypointGraphic", firstWaypoint, 0.02, YoAppearance.White());
      secondWaypointGraphic = new YoGraphicPosition("SecondWaypointGraphic", secondWaypoint, 0.02, YoAppearance.White());
      intersectionMap = new HashMap<>();

      planeNormal = new YoFrameVector3D("planeNormal", ReferenceFrame.getWorldFrame(), registry);
      planeOrigin = new YoFramePoint3D("planeOrigin", ReferenceFrame.getWorldFrame(), registry);
      planeVector = new YoGraphicVector("planeNormal", planeOrigin, planeNormal, 0.2, YoAppearance.Red());

      for (SwingOverPlanarRegionsCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsCollisionType.values())
      {
         AppearanceDefinition appearance;
         double size;
         switch (swingOverPlanarRegionsTrajectoryCollisionType)
         {
         case COLLISION_BETWEEN_FEET:
         case COLLISION_INSIDE_TRAJECTORY:
            appearance = YoAppearance.Red();
            size = 0.014;
            break;
         case OUTSIDE_TRAJECTORY:
            appearance = YoAppearance.Orange();
            size = 0.013;
            break;
         case TOO_CLOSE_TO_IGNORE_PLANE:
            appearance = YoAppearance.Yellow();
            size = 0.012;
            break;
         case NO_INTERSECTION:
            appearance = YoAppearance.Blue();
            size = 0.011;
            break;
         default:
            appearance = YoAppearance.Black();
            size = 0.01;
            break;
         }
         intersectionMap.put(swingOverPlanarRegionsTrajectoryCollisionType,
                             new YoGraphicPosition("IntersectionGraphic" + swingOverPlanarRegionsTrajectoryCollisionType.name(),
                                                   new YoFramePoint3D("IntersectionPoint" + swingOverPlanarRegionsTrajectoryCollisionType.name(), WORLD,
                                                                      registry), size, appearance));

         yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", intersectionMap.get(swingOverPlanarRegionsTrajectoryCollisionType));
      }

      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", trajectoryPosition);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", planeVector);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", actualFootGraphicBox);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", collisionGraphicBox);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingFoot);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", stanceFootGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingStartGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingEndGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", firstWaypointGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", secondWaypointGraphic);
   }

   public void setupSCSGraphs()
   {
      if (scs.getGUI().getGraphArrayPanel().getGraphsOnThisPanel().isEmpty())
      {
         scs.setupGraph("t");
      }
      scs.setupGraph(trajectoryExpander.getNumberOfTriesCounter().getYoCount().getName());
      scs.setupGraph(trajectoryExpander.getDistanceToCollision().getName());
      scs.setupGraph(trajectoryExpander.getMostSevereCollisionType().getName());
      scs.getGUI().getGraphArrayPanel().addColumn();
      scs.setupGraph(trajectoryExpander.getCollisionIsOnRising().getName());
      scs.setupGraph(trajectoryExpander.getHeightAboveFloorPlane().getName());
      scs.setupGraph(trajectoryExpander.getHeightAboveEndFoot().getName());
      scs.setupGraph(trajectoryExpander.getWereWaypointsAdjusted().getName());
      scs.setupEntryBox(trajectoryExpander.getDoInitialFastApproximation().getName());
      scs.setupEntryBox(trajectoryExpander.getNumberOfCheckpoints().getName());
      scs.setupEntryBox(trajectoryExpander.getNumberOfTriesCounter().getYoMaxCount().getName());
      scs.setupEntryBox(trajectoryExpander.getFastApproximationLessClearance().getName());
      scs.setupEntryBox(trajectoryExpander.getMinimumAdjustmentIncrementDistance().getName());
      scs.setupEntryBox(trajectoryExpander.getMaximumAdjustmentIncrementDistance().getName());
      scs.setupEntryBox(trajectoryExpander.getAdjustmentIncrementDistanceGain().getName());
      scs.setupEntryBox(trajectoryExpander.getMaximumAdjustmentDistance().getName());
      scs.setupEntryBox(trajectoryExpander.getMinimumHeightAboveFloorForCollision().getName());
   }

   public void update()
   {
      solePose.setFromReferenceFrame(trajectoryExpander.getSolePoseReferenceFrame());

      for (SwingOverPlanarRegionsCollisionType collisionType : SwingOverPlanarRegionsCollisionType.values())
      {
         intersectionMap.get(collisionType).setPosition(trajectoryExpander.getClosestPolygonPoint(collisionType));
      }

      double footHeight = trajectoryExpander.getFootHeight();

      swingFootPoint.set(solePose.getPosition());
//      swingFootPoint.addZ(0.5 * footHeight);

      swingFoot.update();

      actualFootBoxOrientation.set(trajectoryExpander.getCollisionBox().getOrientation());
      actualFootBoxPoint.set(trajectoryExpander.getCollisionBox().getPosition());
      actualFootGraphicBox.update();

      actualFootBoxReferenceFrame.setPoseAndUpdate(trajectoryExpander.getCollisionBox().getPosition(), trajectoryExpander.getCollisionBox().getOrientation());
      collisionBoxPose.setToZero(actualFootBoxReferenceFrame);
      collisionBoxPose.appendTranslation(0.0, 0.0, -trajectoryExpander.getMinimumClearance());
      collisionBoxPose.changeFrame(WORLD);
      collisionBoxOrientation.set(collisionBoxPose.getOrientation());
      collisionBoxPoint.set(collisionBoxPose.getPosition());
      collisionGraphicBox.update();

      firstWaypoint.set(trajectoryExpander.getExpandedWaypoints().get(0));
      secondWaypoint.set(trajectoryExpander.getExpandedWaypoints().get(1));

      planeOrigin.set(trajectoryExpander.getSwingFloorPlane().getPoint());
      planeNormal.set(trajectoryExpander.getSwingFloorPlane().getNormal());
      swingStartGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), trajectoryExpander.getStartPose()));
      swingEndGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), trajectoryExpander.getEndPose()));

      swingStartGraphic.updateConvexPolygon2d(footPolygon);
      swingEndGraphic.updateConvexPolygon2d(footPolygon);

//      swingFloorPlane.set(swingStartPosition, tempPlaneNormal);


      scs.tickAndUpdate(scs.getTime() + 0.1);
   }

   /*
   public void updateSwingFoot(FramePose3D swingFootPose)
   {
      Point3D frontLeft = new Point3D(trajectoryExpander.getToeLength(), 0.5 * trajectoryExpander.getFootWidth(), 0.0);
      Point3D frontRight = new Point3D(trajectoryExpander.getToeLength(), -0.5 * trajectoryExpander.getFootWidth(), 0.0);
      Point3D hindLeft = new Point3D(-trajectoryExpander.getHeelLength(), 0.5 * trajectoryExpander.getFootWidth(), 0.0);
      Point3D hindRight = new Point3D(-trajectoryExpander.getHeelLength(), -0.5 * trajectoryExpander.getFootWidth(), 0.0);

      RigidBodyTransform footTransform = new RigidBodyTransform();
      swingFootPose.get(footTransform);
      footTransform.transform(frontLeft);
      footTransform.transform(frontRight);
      footTransform.transform(hindRight);
      footTransform.transform(hindLeft);

      List<Point3D> footVertices = new ArrayList<>();
      footVertices.add(frontLeft);
      footVertices.add(frontRight);
      footVertices.add(hindLeft);
      footVertices.add(hindRight);

      swingFoot.setPose(swingFootPose);


      double clearance = trajectoryExpander.getMinimumClearance();
      frontLeft = new Point3D(trajectoryExpander.getToeLength() + clearance, 0.5 * trajectoryExpander.getFootWidth() + clearance, 0.0);
      frontRight = new Point3D(trajectoryExpander.getToeLength() + clearance, -0.5 * trajectoryExpander.getFootWidth() - clearance, 0.0);
      hindLeft = new Point3D(-trajectoryExpander.getHeelLength() - clearance, 0.5 * trajectoryExpander.getFootWidth()  + clearance, 0.0);
      hindRight = new Point3D(-trajectoryExpander.getHeelLength() - clearance, -0.5 * trajectoryExpander.getFootWidth() - clearance, 0.0);
      footTransform.transform(frontLeft);
      footTransform.transform(frontRight);
      footTransform.transform(hindRight);
      footTransform.transform(hindLeft);

      footVertices = new ArrayList<>();
      footVertices.add(frontLeft);
      footVertices.add(frontRight);
      footVertices.add(hindLeft);
      footVertices.add(hindRight);

      collisionBox.set
      collisionBox.set(footVertices);
   }
   */

   public void updateFoot(FramePose3D stanceFootPose, FramePose3D swingStartPose, FramePose3D swingEndPose)
   {
      stanceFootGraphic.setPose(stanceFootPose);
      stanceFootGraphic.updateConvexPolygon2d(footPolygon);
      swingStartGraphic.setPose(swingStartPose);
      swingStartGraphic.updateConvexPolygon2d(footPolygon);
      swingEndGraphic.setPose(swingEndPose);
      swingEndGraphic.updateConvexPolygon2d(footPolygon);
   }
}
