package us.ihmc.robotEnvironmentAwareness.slam;

import us.ihmc.jOctoMap.tools.ScannerTools;

import java.util.Scanner;

/**
 * @see <a href="https://docs.google.com/presentation/d/1GH2QqMLM1cKgll5hcd-nX18c-cGVjB5ABkz_voOZU1w/edit?usp=sharing">20200722_LearningLunch_Inho</a>
 */
public class SurfaceElementICPSLAMParameters
{
   private static final double DEFAULT_SURFACE_ELEMENT_RESOLUTION = 0.04;
   private static final double DEFAULT_WINDOW_MARGIN = 0.0;
   private static final int DEFAULT_MINIMUM_NUMBER_OF_HIT = 1;
   private static final double DEFAULT_BOUND_RATIO = 1.1;

   private static final double DEFAULT_MINIMUM_CORRESPONDING_DISTANCE = 0.06;
   private static final double DEFAULT_MAXIMUM_CORRESPONDING_DISTANCE = 0.1;
   private static final int DEFAULT_CONSTANT_CORRESPONDING_DISTANCE_ITERATION = 3;

   private static final int DEFAULT_STEADY_STATE_DETECTOR_ITERATION_THRESHOLD = 3;
   private static final double DEFAULT_QUALITY_CONVERGENCE_THRESHOLD = 0.001;
   private static final double DEFAULT_TRANSLATIONAL_EFFORT_CONVERGENCE_THRESHOLD = 0.001;
   private static final double DEFAULT_ROTATIONAL_EFFORT_CONVERGENCE_THRESHOLD = 0.005;

   private static final boolean DEFAULT_ENABLE_INITIAL_QUALITY_FILTER = false;
   private static final double DEFAULT_INITIAL_QUALITY_THRESHOLD = 0.1;

   private static final int DEFAULT_MAX_OPTIMIZATION_ITERATIONS = 40;
   private static final boolean DEFAULT_COMPUTE_SURFACE_NORMALS_IN_FRAME = true;

   private static final boolean DEFAULT_INSERT_MISS_IN_OCTREE = true;

   private static final int DEFAULT_MAXIMUM_QUEUE_SIZE = Integer.MAX_VALUE;
   private static final double DEFAULT_MAXIMUM_TIME_BETWEEN_FRAMES = 1.0;
   private static final double DEFAULT_LONGEST_TIME_TO_LAG = Double.POSITIVE_INFINITY;

   private double surfaceElementResolution;
   private double windowMargin;
   private int minimumNumberOfHit;
   private double boundRatio;

   private double minimumCorrespondingDistance;
   private double maximumCorrespondingDistance;
   private int constantCorrespondingDistanceIteration;

   private int steadyStateDetectorIterationThreshold;
   private double qualityConvergenceThreshold;
   private double translationalEffortConvergenceThreshold;
   private double rotationalEffortConvergenceThreshold;

   private boolean enableInitialQualityFilter;
   private double initialQualityThreshold;

   private int maxOptimizationIterations;
   private boolean computeSurfaceNormalsInFrame;

   private boolean insertMissInOcTree;

   private int maximumQueueSize;
   private double maximumTimeBetweenFrames;
   private double longestTimeToLag;

   public SurfaceElementICPSLAMParameters()
   {
      setDefaultParameters();
   }

   public SurfaceElementICPSLAMParameters(SurfaceElementICPSLAMParameters other)
   {
      set(other);
   }

   public void set(SurfaceElementICPSLAMParameters other)
   {
      surfaceElementResolution = other.surfaceElementResolution;
      windowMargin = other.windowMargin;
      minimumNumberOfHit = other.minimumNumberOfHit;
      boundRatio = other.boundRatio;

      minimumCorrespondingDistance = other.minimumCorrespondingDistance;
      maximumCorrespondingDistance = other.maximumCorrespondingDistance;
      constantCorrespondingDistanceIteration = other.constantCorrespondingDistanceIteration;

      steadyStateDetectorIterationThreshold = other.steadyStateDetectorIterationThreshold;
      qualityConvergenceThreshold = other.qualityConvergenceThreshold;
      translationalEffortConvergenceThreshold = other.translationalEffortConvergenceThreshold;
      rotationalEffortConvergenceThreshold = other.rotationalEffortConvergenceThreshold;

      enableInitialQualityFilter = other.enableInitialQualityFilter;
      initialQualityThreshold = other.initialQualityThreshold;

      maxOptimizationIterations = other.maxOptimizationIterations;
      computeSurfaceNormalsInFrame = other.computeSurfaceNormalsInFrame;

      insertMissInOcTree = other.insertMissInOcTree;
      maximumQueueSize = other.maximumQueueSize;
      maximumTimeBetweenFrames = other.maximumTimeBetweenFrames;
      longestTimeToLag = other.longestTimeToLag;
   }

   public void setDefaultParameters()
   {
      surfaceElementResolution = DEFAULT_SURFACE_ELEMENT_RESOLUTION;
      windowMargin = DEFAULT_WINDOW_MARGIN;
      minimumNumberOfHit = DEFAULT_MINIMUM_NUMBER_OF_HIT;
      boundRatio = DEFAULT_BOUND_RATIO;

      minimumCorrespondingDistance = DEFAULT_MINIMUM_CORRESPONDING_DISTANCE;
      maximumCorrespondingDistance = DEFAULT_MAXIMUM_CORRESPONDING_DISTANCE;
      constantCorrespondingDistanceIteration = DEFAULT_CONSTANT_CORRESPONDING_DISTANCE_ITERATION;

      steadyStateDetectorIterationThreshold = DEFAULT_STEADY_STATE_DETECTOR_ITERATION_THRESHOLD;
      qualityConvergenceThreshold = DEFAULT_QUALITY_CONVERGENCE_THRESHOLD;
      translationalEffortConvergenceThreshold = DEFAULT_TRANSLATIONAL_EFFORT_CONVERGENCE_THRESHOLD;
      rotationalEffortConvergenceThreshold = DEFAULT_ROTATIONAL_EFFORT_CONVERGENCE_THRESHOLD;

      enableInitialQualityFilter = DEFAULT_ENABLE_INITIAL_QUALITY_FILTER;
      initialQualityThreshold = DEFAULT_INITIAL_QUALITY_THRESHOLD;

      maxOptimizationIterations = DEFAULT_MAX_OPTIMIZATION_ITERATIONS;
      computeSurfaceNormalsInFrame = DEFAULT_COMPUTE_SURFACE_NORMALS_IN_FRAME;

      insertMissInOcTree = DEFAULT_INSERT_MISS_IN_OCTREE;
      maximumQueueSize = DEFAULT_MAXIMUM_QUEUE_SIZE;
      maximumTimeBetweenFrames = DEFAULT_MAXIMUM_TIME_BETWEEN_FRAMES;
      longestTimeToLag = DEFAULT_LONGEST_TIME_TO_LAG;
   }

   public double getSurfaceElementResolution()
   {
      return surfaceElementResolution;
   }

   public double getWindowMargin()
   {
      return windowMargin;
   }

   public int getMinimumNumberOfHit()
   {
      return minimumNumberOfHit;
   }

   public double getBoundRatio()
   {
      return boundRatio;
   }

   public double getMinimumCorrespondingDistance()
   {
      return minimumCorrespondingDistance;
   }

   public double getMaximumCorrespondingDistance()
   {
      return maximumCorrespondingDistance;
   }

   public int getConstantCorrespondingDistanceIteration()
   {
      return constantCorrespondingDistanceIteration;
   }

   public int getSteadyStateDetectorIterationThreshold()
   {
      return steadyStateDetectorIterationThreshold;
   }

   public double getQualityConvergenceThreshold()
   {
      return qualityConvergenceThreshold;
   }

   public double getTranslationalEffortConvergenceThreshold()
   {
      return translationalEffortConvergenceThreshold;
   }

   public double getRotationalEffortConvergenceThreshold()
   {
      return rotationalEffortConvergenceThreshold;
   }

   public boolean isEnableInitialQualityFilter()
   {
      return enableInitialQualityFilter;
   }

   public double getInitialQualityThreshold()
   {
      return initialQualityThreshold;
   }

   public int getMaxOptimizationIterations()
   {
      return maxOptimizationIterations;
   }

   public boolean getComputeSurfaceNormalsInFrame()
   {
      return computeSurfaceNormalsInFrame;
   }

   public boolean getInsertMissInOcTree()
   {
      return insertMissInOcTree;
   }

   public int getMaximumQueueSize()
   {
      return maximumQueueSize;
   }

   public double getMaximumTimeBetweenFrames()
   {
      return maximumTimeBetweenFrames;
   }

   public double getLongestTimeToLag()
   {
      return longestTimeToLag;
   }

   public void setSurfaceElementResolution(double surfaceElementResolution)
   {
      this.surfaceElementResolution = surfaceElementResolution;
   }

   public void setWindowMargin(double windowMargin)
   {
      this.windowMargin = windowMargin;
   }

   public void setMinimumNumberOfHit(int minimumNumberOfHit)
   {
      this.minimumNumberOfHit = minimumNumberOfHit;
   }

   public void setBoundRatio(double boundRatio)
   {
      this.boundRatio = boundRatio;
   }

   public void setMinimumCorrespondingDistance(double minimumCorrespondingDistance)
   {
      this.minimumCorrespondingDistance = minimumCorrespondingDistance;
   }

   public void setMaximumCorrespondingDistance(double maximumCorrespondingDistance)
   {
      this.maximumCorrespondingDistance = maximumCorrespondingDistance;
   }

   public void setConstantCorrespondingDistanceIteration(int constantCorrespondingDistanceIteration)
   {
      this.constantCorrespondingDistanceIteration = constantCorrespondingDistanceIteration;
   }

   public void setSteadyStateDetectorIterationThreshold(int steadyStateDetectorIterationThreshold)
   {
      this.steadyStateDetectorIterationThreshold = steadyStateDetectorIterationThreshold;
   }

   public void setQualityConvergenceThreshold(double qualityConvergenceThreshold)
   {
      this.qualityConvergenceThreshold = qualityConvergenceThreshold;
   }

   public void setTranslationalEffortConvergenceThreshold(double translationalEffortConvergenceThreshold)
   {
      this.translationalEffortConvergenceThreshold = translationalEffortConvergenceThreshold;
   }

   public void setRotationalEffortConvergenceThreshold(double rotationalEffortConvergenceThreshold)
   {
      this.rotationalEffortConvergenceThreshold = rotationalEffortConvergenceThreshold;
   }

   public void setEnableInitialQualityFilter(boolean enableInitialQualityFilter)
   {
      this.enableInitialQualityFilter = enableInitialQualityFilter;
   }

   public void setInitialQualityThreshold(double initialQualityThreshold)
   {
      this.initialQualityThreshold = initialQualityThreshold;
   }

   public void setMaxOptimizationIterations(int maximumCorrespondingDistance)
   {
      this.maxOptimizationIterations = maximumCorrespondingDistance;
   }

   public void setComputeSurfaceNormalsInFrame(boolean computeSurfaceNormalsInFrame)
   {
      this.computeSurfaceNormalsInFrame = computeSurfaceNormalsInFrame;
   }

   public void setInsertMissInOcTree(boolean insertMissInOcTree)
   {
      this.insertMissInOcTree = insertMissInOcTree;
   }

   public void setMaximumQueueSize(int maximumQueueSize)
   {
      this.maximumQueueSize = maximumQueueSize;
   }

   public void setMaximumTimeBetweenFrames(double maximumTimeBetweenFrames)
   {
      this.maximumTimeBetweenFrames = maximumTimeBetweenFrames;
   }

   public void setLongestTimeToLag(double longestTimeToLag)
   {
      this.longestTimeToLag = longestTimeToLag;
   }

   @Override
   public String toString()
   {
      return "surfaceElementResolution: " + getSurfaceElementResolution() + ", windowMargin: " + getWindowMargin() + ", minimumNumberOfHit: "
             + getMinimumNumberOfHit() + ", boundRatio: " + getBoundRatio() + ", minimumCorrespondingDistance: " + getMinimumCorrespondingDistance()
             + ", maximumCorrespondingDistance: " + getMaximumCorrespondingDistance() + ", constantCorrespondingDistanceIteration: "
             + getConstantCorrespondingDistanceIteration() + ", steadStateDetectorIterationThreshold: " + getSteadyStateDetectorIterationThreshold()
             + ", qualityConvergenceThreshold: " + getQualityConvergenceThreshold() + ", translationalEffortConvergenceThreshold: "
             + getTranslationalEffortConvergenceThreshold() + ", rotationalEffortConvergenceThreshold: " + getRotationalEffortConvergenceThreshold()
             + ", enableInitialQualityFilter: " + isEnableInitialQualityFilter() + ", initialQualityThreshold: " + getInitialQualityThreshold()
             + ", maxOptimizationIterations: " + getMaxOptimizationIterations() + " computeSurfaceNormalsInPlane: " + getComputeSurfaceNormalsInFrame()
            + ", insertMissInOcTree: " + getInsertMissInOcTree() + ", maximumQueueSize: " + getMaximumQueueSize() + ", maximumTimeBetweenFrames: "
            + getMaximumTimeBetweenFrames() + ", longestTimeToLag: " + getLongestTimeToLag();
   }

   public static SurfaceElementICPSLAMParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);

      SurfaceElementICPSLAMParameters parameters = new SurfaceElementICPSLAMParameters();
      parameters.setSurfaceElementResolution(ScannerTools.readNextDouble(scanner, parameters.getSurfaceElementResolution()));
      parameters.setWindowMargin(ScannerTools.readNextDouble(scanner, parameters.getWindowMargin()));
      parameters.setMinimumNumberOfHit(ScannerTools.readNextInt(scanner, parameters.getMinimumNumberOfHit()));
      parameters.setBoundRatio(ScannerTools.readNextDouble(scanner, parameters.getBoundRatio()));
      parameters.setMinimumCorrespondingDistance(ScannerTools.readNextDouble(scanner, parameters.getMinimumCorrespondingDistance()));
      parameters.setMaximumCorrespondingDistance(ScannerTools.readNextDouble(scanner, parameters.getMaximumCorrespondingDistance()));
      parameters.setConstantCorrespondingDistanceIteration(ScannerTools.readNextInt(scanner, parameters.getConstantCorrespondingDistanceIteration()));
      parameters.setSteadyStateDetectorIterationThreshold(ScannerTools.readNextInt(scanner, parameters.getSteadyStateDetectorIterationThreshold()));
      parameters.setQualityConvergenceThreshold(ScannerTools.readNextDouble(scanner, parameters.getQualityConvergenceThreshold()));
      parameters.setTranslationalEffortConvergenceThreshold(ScannerTools.readNextDouble(scanner, parameters.getTranslationalEffortConvergenceThreshold()));
      parameters.setRotationalEffortConvergenceThreshold(ScannerTools.readNextDouble(scanner, parameters.getRotationalEffortConvergenceThreshold()));
      parameters.setEnableInitialQualityFilter(ScannerTools.readNextBoolean(scanner, parameters.isEnableInitialQualityFilter()));
      parameters.setInitialQualityThreshold(ScannerTools.readNextDouble(scanner, parameters.getInitialQualityThreshold()));
      parameters.setMaxOptimizationIterations(ScannerTools.readNextInt(scanner, parameters.getMaxOptimizationIterations()));
      parameters.setComputeSurfaceNormalsInFrame(ScannerTools.readNextBoolean(scanner, parameters.getComputeSurfaceNormalsInFrame()));
      parameters.setInsertMissInOcTree(ScannerTools.readNextBoolean(scanner, parameters.getInsertMissInOcTree()));
      parameters.setMaximumQueueSize(ScannerTools.readNextInt(scanner, parameters.getMaximumQueueSize()));
      parameters.setMaximumTimeBetweenFrames(ScannerTools.readNextDouble(scanner, parameters.getMaximumTimeBetweenFrames()));
      parameters.setLongestTimeToLag(ScannerTools.readNextDouble(scanner, parameters.getLongestTimeToLag()));
      scanner.close();
      return parameters;
   }
}
