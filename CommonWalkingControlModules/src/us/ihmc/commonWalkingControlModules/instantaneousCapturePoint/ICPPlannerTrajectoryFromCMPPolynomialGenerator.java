package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPPlannerTrajectoryFromCMPPolynomialGenerator implements PositionTrajectoryGenerator
{
   private final DoubleYoVariable omega0;
   private ReferenceFrame trajectoryFrame;
   
   private final static int firstSegment = 0;
   private final static int X = Direction.X.ordinal();
   private final static int Y = Direction.Y.ordinal();
   private final static int Z = Direction.Z.ordinal();
   
   private final List<YoPolynomial3D> cmpPolynomialTrajectories = new ArrayList<>();
   
   private final List<FramePoint> desiredICPBoundaryPositions = new ArrayList<>();
   
   private final FramePoint icpPositionDesiredOutput = new FramePoint();
   private final FrameVector icpVelocityDesiredOutput = new FrameVector();
   private final FrameVector icpAccelerationDesiredOutput = new FrameVector();
   
   private final DenseMatrix64F coefficientsCombinedVector = new DenseMatrix64F();
   private final DenseMatrix64F coefficientsCurrentVector = new DenseMatrix64F();
   
   Point3D icpPositionDesiredCurrent = new Point3D();
   Point3D icpPositionDesiredFinal = new Point3D();
   Vector3D icpVelocityDesiredCurrent = new Vector3D();
   Vector3D icpAccelerationDesiredCurrent = new Vector3D();
   
   private DenseMatrix64F alphaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F betaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F gammaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F alphaBetaPrimeMatrix = new DenseMatrix64F();
   
   private DenseMatrix64F dAlphaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F dBetaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F dGammaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F dAlphaBetaPrimeMatrix = new DenseMatrix64F();
   
   private DenseMatrix64F ddAlphaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F ddBetaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F ddGammaPrimeMatrix = new DenseMatrix64F();
   private DenseMatrix64F ddAlphaBetaPrimeMatrix = new DenseMatrix64F();
   
   private DenseMatrix64F identityMatrix = new DenseMatrix64F();
   private DenseMatrix64F backwardIterationMatrix = new DenseMatrix64F();
   private DenseMatrix64F finalConstraintMatrix = new DenseMatrix64F();
   private DenseMatrix64F tPowersMatrix = new DenseMatrix64F();
   private DenseMatrix64F polynomialCoefficientVector = new DenseMatrix64F();
   
   private final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(10000, 1);
   private final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(10000, 1);

   
   private DenseMatrix64F icpPositionDesiredInitialMatrix = new DenseMatrix64F(10000, 3);
   private DenseMatrix64F icpPositionDesiredFinalMatrix = new DenseMatrix64F(10000, 3);
   
   // Pre-allocated helper matrices
   private final DenseMatrix64F M1 = new DenseMatrix64F(10000, 10000);
   private final DenseMatrix64F M2 = new DenseMatrix64F(10000, 10000);
   private final DenseMatrix64F M3 = new DenseMatrix64F(10000, 10000);
   
   private final int numberOfSegments;
      
   public ICPPlannerTrajectoryFromCMPPolynomialGenerator(DoubleYoVariable omega0, ReferenceFrame trajectoryFrame, List<YoPolynomial3D> cmpPolynomialTrajectories)
   {
      this.omega0 = omega0;
      this.trajectoryFrame = trajectoryFrame;
      
      numberOfSegments = cmpPolynomialTrajectories.size();
      
      icpPositionDesiredInitialMatrix.reshape(numberOfSegments, 3);
      icpPositionDesiredFinalMatrix.reshape(numberOfSegments, 3);
      
      
      
      for(int i = 0; i < cmpPolynomialTrajectories.size(); ++i)
      {
         this.cmpPolynomialTrajectories.add(cmpPolynomialTrajectories.get(i));
      }
   }
   
   public void initialize()
   {
      calculateICPDesiredBoundaryValuesRecursivelyFromCMPPolynomialScalar();
   }

   public void compute(double time)
   {
      initialize();
      calculateICPPositionDesiredCurrentFromCMPPolynomialsScalar(time);
      calculateICPVelocityDesiredCurrentFromCMPPolynomialsScalar(time);
      calculateICPAccelerationDesiredCurrentFromCMPPolynomialsScalar(time);
   }
   
   // desiredICPCurrent: desired ICP at time; modified.
   // TODO: take care of frame matching
//   private void calculateICPFromCorrespondingCMPPolynomial(YoFramePoint desiredICPCurrent, YoFramePoint desiredICPFinal, YoPolynomial3D cmpPolynomialTrajectory, double time, double trajectoryTime)
//   {
//      Point3D desiredICPCurrentPosition = new Point3D();
//      desiredICPCurrent.getPoint(desiredICPCurrentPosition);
//      
//      Point3D desiredICPFinalPosition = new Point3D();
//      desiredICPFinal.getPoint(desiredICPFinalPosition);
//      
//      calculateICPFromCorrespondingCMPPolynomial(desiredICPCurrentPosition, desiredICPFinalPosition, cmpPolynomialTrajectory, time, trajectoryTime);
//      
//      desiredICPCurrent.setPoint(desiredICPCurrentPosition);
//   }
//   
//   private void calculateICPFromCorrespondingCMPPolynomial(Point3D desiredICPCurrent, Point3D desiredICPFinal, YoPolynomial3D cmpPolynomialTrajectory, double time, double trajectoryTime)
//   {     
//      double desiredICPX = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getX(), cmpPolynomialTrajectory.getYoPolynomialX(), time, trajectoryTime);
//      double desiredICPY = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getY(), cmpPolynomialTrajectory.getYoPolynomialY(), time, trajectoryTime);
//      double desiredICPZ = calculateICPFromCorrespondingCMPPolynomial1D(desiredICPCurrent.getZ(), cmpPolynomialTrajectory.getYoPolynomialZ(), time, trajectoryTime);
//      
//      desiredICPCurrent.set(new Point3D(desiredICPX, desiredICPY, desiredICPZ));
//   }
//   
//   private double calculateICPFromCorrespondingCMPPolynomial1D(double desiredICPFinalPosition1D, YoPolynomial cmpPolynomialTrajectory1D, double time, double trajectoryTime)
//   {      
//      double exponentialFactor = Math.exp(omega0.getDoubleValue()*(time-trajectoryTime));
//      
//      double desiredICPCurrentPosition1D = desiredICPFinalPosition1D * exponentialFactor;  
//      for(int i = 0; i < cmpPolynomialTrajectory1D.getNumberOfCoefficients(); ++i)
//      {
//         desiredICPCurrentPosition1D += 1/Math.pow(omega0.getDoubleValue(), i) * (cmpPolynomialTrajectory1D.getDerivative(i, time) - cmpPolynomialTrajectory1D.getDerivative(i, trajectoryTime) * exponentialFactor);
//      }
//      
//      return desiredICPCurrentPosition1D;
//   }
   
   
   // SCALAR FORMULATION
   private void calculateICPPositionDesiredCurrentFromCMPPolynomialsScalar(double time)
   {            
      icpPositionDesiredCurrent.setX(calculateICPPositionFromCorrespondingCMPPolynomialScalarX(icpPositionDesiredFinal.getX(), firstSegment, time));
      icpPositionDesiredCurrent.setY(calculateICPPositionFromCorrespondingCMPPolynomialScalarY(icpPositionDesiredFinal.getY(), firstSegment, time)); 
      icpPositionDesiredCurrent.setZ(calculateICPPositionFromCorrespondingCMPPolynomialScalarZ(icpPositionDesiredFinal.getZ(), firstSegment, time));
      
      icpPositionDesiredOutput.setIncludingFrame(trajectoryFrame, icpPositionDesiredCurrent);
   }
   
   private void calculateICPVelocityDesiredCurrentFromCMPPolynomialsScalar(double time)
   {            
      icpVelocityDesiredCurrent.setX(calculateICPVelocityFromCorrespondingCMPPolynomialScalarX(icpPositionDesiredFinal.getX(), firstSegment, time));
      icpVelocityDesiredCurrent.setY(calculateICPVelocityFromCorrespondingCMPPolynomialScalarY(icpPositionDesiredFinal.getY(), firstSegment, time)); 
      icpVelocityDesiredCurrent.setZ(calculateICPVelocityFromCorrespondingCMPPolynomialScalarZ(icpPositionDesiredFinal.getZ(), firstSegment, time));
      
      icpVelocityDesiredOutput.setIncludingFrame(trajectoryFrame, icpVelocityDesiredCurrent);
   }
   
   private void calculateICPAccelerationDesiredCurrentFromCMPPolynomialsScalar(double time)
   {            
      icpAccelerationDesiredCurrent.setX(calculateICPAccelerationFromCorrespondingCMPPolynomialScalarX(icpPositionDesiredFinal.getX(), firstSegment, time));
      icpAccelerationDesiredCurrent.setY(calculateICPAccelerationFromCorrespondingCMPPolynomialScalarY(icpPositionDesiredFinal.getY(), firstSegment, time)); 
      icpAccelerationDesiredCurrent.setZ(calculateICPAccelerationFromCorrespondingCMPPolynomialScalarZ(icpPositionDesiredFinal.getZ(), firstSegment, time));
      
      icpAccelerationDesiredOutput.setIncludingFrame(trajectoryFrame, icpAccelerationDesiredCurrent);
   }
   
   private void calculateICPDesiredBoundaryValuesRecursivelyFromCMPPolynomialScalar()
   {       
      setICPTerminalConditionScalar();
      
      for(int i = numberOfSegments-1; i >= 0; i--)
      {
         icpPositionDesiredInitialMatrix.set(i, X, calculateICPPositionFromCorrespondingCMPPolynomialScalarX(icpPositionDesiredFinalMatrix.get(i, X), i, 0.0));
         icpPositionDesiredInitialMatrix.set(i, Y, calculateICPPositionFromCorrespondingCMPPolynomialScalarY(icpPositionDesiredFinalMatrix.get(i, Y), i, 0.0));
         icpPositionDesiredInitialMatrix.set(i, Z, calculateICPPositionFromCorrespondingCMPPolynomialScalarZ(icpPositionDesiredFinalMatrix.get(i, Z), i, 0.0));
         
         if(i > 0)
         {
            icpPositionDesiredFinalMatrix.set(i-1, X, icpPositionDesiredInitialMatrix.get(i, X));
            icpPositionDesiredFinalMatrix.set(i-1, Y, icpPositionDesiredInitialMatrix.get(i, Y));
            icpPositionDesiredFinalMatrix.set(i-1, Z, icpPositionDesiredInitialMatrix.get(i, Z));
         }
      }
      icpPositionDesiredFinal.set(icpPositionDesiredFinalMatrix.get(firstSegment, X), icpPositionDesiredFinalMatrix.get(firstSegment, Y), icpPositionDesiredFinalMatrix.get(firstSegment, Z));
   }
   
   private void setICPTerminalConditionScalar()
   {
      for(int i = 0; i < 3; i++)
      {
         YoPolynomial cmPolynomialFinalSegment = cmpPolynomialTrajectories.get(numberOfSegments-1).getYoPolynomial(i);
         cmPolynomialFinalSegment.compute(cmPolynomialFinalSegment.getXFinal());
         icpPositionDesiredFinalMatrix.set(numberOfSegments-1, i, cmPolynomialFinalSegment.getPosition());
      }
   }
   
   
   
   
   private double calculateICPPositionFromCorrespondingCMPPolynomialScalarX(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();

      return calculateICPPositionFromCorrespondingCMPPolynomialScalar(cmpPolynomialX, icpDesiredFinal, time);
   }
   
   private double calculateICPPositionFromCorrespondingCMPPolynomialScalarY(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();

      return calculateICPPositionFromCorrespondingCMPPolynomialScalar(cmpPolynomialY, icpDesiredFinal, time);
   }
   
   private double calculateICPPositionFromCorrespondingCMPPolynomialScalarZ(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();

      return calculateICPPositionFromCorrespondingCMPPolynomialScalar(cmpPolynomialZ, icpDesiredFinal, time);
   }
   
   private double calculateICPPositionFromCorrespondingCMPPolynomialScalar(YoPolynomial cmpPolynomial, double icpPositionDesiredFinal, double time)
   {
      double timeTrajectory = cmpPolynomial.getXFinal();
      DenseMatrix64F polynomialCoefficientVector =  cmpPolynomial.getCoefficientsVector();
      
      calculateAlphaPrimeOnCMPSegment(alphaPrimeMatrix, cmpPolynomial, time);
      calculateBetaPrimeOnCMPSegment(betaPrimeMatrix, cmpPolynomial, time, timeTrajectory);
      calculateGammaPrimeOnCMPSegment(gammaPrimeMatrix, time, timeTrajectory);
      CommonOps.subtract(alphaPrimeMatrix, betaPrimeMatrix, alphaBetaPrimeMatrix);
      
      return calculateICPQuantityFromCorrespondingCMPPolynomialScalar(alphaBetaPrimeMatrix, gammaPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal);
   }
   
   private double calculateICPVelocityFromCorrespondingCMPPolynomialScalarX(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();

      return calculateICPVelocityFromCorrespondingCMPPolynomialScalar(cmpPolynomialX, icpDesiredFinal, time);
   }
   
   private double calculateICPVelocityFromCorrespondingCMPPolynomialScalarY(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();

      return calculateICPVelocityFromCorrespondingCMPPolynomialScalar(cmpPolynomialY, icpDesiredFinal, time);
   }
   
   private double calculateICPVelocityFromCorrespondingCMPPolynomialScalarZ(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();

      return calculateICPVelocityFromCorrespondingCMPPolynomialScalar(cmpPolynomialZ, icpDesiredFinal, time);
   }
   
   private double calculateICPVelocityFromCorrespondingCMPPolynomialScalar(YoPolynomial cmpPolynomial, double icpPositionDesiredFinal, double time)
   {
      double timeTrajectory = cmpPolynomial.getXFinal();
      DenseMatrix64F polynomialCoefficientVector =  cmpPolynomial.getCoefficientsVector();
      
      calculateDAlphaPrimeOnCMPSegment(dAlphaPrimeMatrix, cmpPolynomial, time);
      calculateDBetaPrimeOnCMPSegment(dBetaPrimeMatrix, cmpPolynomial, time, timeTrajectory);
      calculateDGammaPrimeOnCMPSegment(dGammaPrimeMatrix, time, timeTrajectory);
      CommonOps.subtract(dAlphaPrimeMatrix, dBetaPrimeMatrix, dAlphaBetaPrimeMatrix);
      
      return calculateICPQuantityFromCorrespondingCMPPolynomialScalar(dAlphaBetaPrimeMatrix, dGammaPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal);
   }
   
   private double calculateICPAccelerationFromCorrespondingCMPPolynomialScalarX(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();

      return calculateICPAccelerationFromCorrespondingCMPPolynomialScalar(cmpPolynomialX, icpDesiredFinal, time);
   }
   
   private double calculateICPAccelerationFromCorrespondingCMPPolynomialScalarY(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();

      return calculateICPAccelerationFromCorrespondingCMPPolynomialScalar(cmpPolynomialY, icpDesiredFinal, time);
   }
   
   private double calculateICPAccelerationFromCorrespondingCMPPolynomialScalarZ(double icpDesiredFinal, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();

      return calculateICPAccelerationFromCorrespondingCMPPolynomialScalar(cmpPolynomialZ, icpDesiredFinal, time);
   }
   
   private double calculateICPAccelerationFromCorrespondingCMPPolynomialScalar(YoPolynomial cmpPolynomial, double icpPositionDesiredFinal, double time)
   {      
      double timeTrajectory = cmpPolynomial.getXFinal();
      DenseMatrix64F polynomialCoefficientVector =  cmpPolynomial.getCoefficientsVector();
      
      calculateDDAlphaPrimeOnCMPSegment(ddAlphaPrimeMatrix, cmpPolynomial, time);
      calculateDDBetaPrimeOnCMPSegment(ddBetaPrimeMatrix, cmpPolynomial, time, timeTrajectory);
      calculateDDGammaPrimeOnCMPSegment(ddGammaPrimeMatrix, time, timeTrajectory);
      CommonOps.subtract(ddAlphaPrimeMatrix, ddBetaPrimeMatrix, ddAlphaBetaPrimeMatrix);
      
      return calculateICPQuantityFromCorrespondingCMPPolynomialScalar(ddAlphaBetaPrimeMatrix, ddGammaPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal);
   }
   
   private double calculateICPQuantityFromCorrespondingCMPPolynomialScalar(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, DenseMatrix64F generalizedGammaPrimeMatrix,
                                                                           DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal)
   {
      double icpQuantityDesired = 0.0;
      
      M1.reshape(generalizedAlphaBetaPrimeMatrix.getNumRows(), generalizedAlphaBetaPrimeMatrix.getNumCols());
      M1.zero();
      M2.reshape(generalizedGammaPrimeMatrix.getNumRows(), generalizedGammaPrimeMatrix.getNumCols());
      M2.zero();
      
      CommonOps.mult(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientVector, M1);
      CommonOps.scale(icpPositionDesiredFinal, generalizedGammaPrimeMatrix, M2);
      
      CommonOps.addEquals(M1, M2);
      
      icpQuantityDesired = M1.get(0, 0);
      return icpQuantityDesired;
   }
   
   // MATRIX FORMULATION
   // ICP = (M1)^(-1) * M2 * P = M3 * P
   private void calculateICPFromCorrespondingCMPPolynomialMatrix(DenseMatrix64F icpVector, DenseMatrix64F identityMatrix, DenseMatrix64F gammaPrimeMatrix, DenseMatrix64F backwardIterationMatrix,
                                                                DenseMatrix64F alphaBetaPrimeMatrix, DenseMatrix64F terminalConstraintMatrix, DenseMatrix64F tPowersVector, DenseMatrix64F polynomialCoefficientVector)
   {
      M1.reshape(identityMatrix.getNumRows(), identityMatrix.getNumCols());
      M1.zero();
      
      M2.reshape(alphaBetaPrimeMatrix.getNumRows(), alphaBetaPrimeMatrix.getNumCols());
      M2.zero();
      
      M3.reshape(identityMatrix.getNumRows(), alphaBetaPrimeMatrix.getNumCols());
      M3.zero();
      
      CommonOps.mult(gammaPrimeMatrix, backwardIterationMatrix, M1);
      CommonOps.subtract(identityMatrix, M1, M1);
      
      CommonOps.mult(terminalConstraintMatrix, tPowersVector, M2);
      CommonOps.mult(gammaPrimeMatrix, M2, M2);
      CommonOps.addEquals(M2, alphaBetaPrimeMatrix);
      
      CommonOps.invert(M1, M3);
      CommonOps.mult(M3, M2, M3);
      
      CommonOps.mult(M3, polynomialCoefficientVector, icpVector);
   }
   
   
   private void calculateAlphaPrimeMatrixOnSegment(DenseMatrix64F alphaPrimeMatrix, int segment, double time)
   {
      
   }
   
   private void calculateBetaPrimeMatrixOnSegment(DenseMatrix64F betaPrimeMatrix, int segment, double time)
   {
      
   }
   
   private void calculateGammaPrimeMatrixOnSegment(DenseMatrix64F gammaPrimeMatrix, int segment, double time)
   {
      
   }
   
   
   
   private void calculateAlphaPrimeOnCMPSegmentX(DenseMatrix64F alphaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();
      
      calculateAlphaPrimeOnCMPSegment(alphaPrime, cmpPolynomialX, time);
   }
   
   private void calculateAlphaPrimeOnCMPSegmentY(DenseMatrix64F alphaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();
      
      calculateAlphaPrimeOnCMPSegment(alphaPrime, cmpPolynomialY, time);
   }
   
   private void calculateAlphaPrimeOnCMPSegmentZ(DenseMatrix64F alphaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();
      
      calculateAlphaPrimeOnCMPSegment(alphaPrime, cmpPolynomialZ, time);
   }
   
   
   private void calculateAlphaPrimeOnCMPSegment(DenseMatrix64F alphaPrime, YoPolynomial cmpPolynomial, double time)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i);
         CommonOps.addEquals(alphaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateDAlphaPrimeOnCMPSegment(DenseMatrix64F dAlphaPrime, YoPolynomial cmpPolynomial, double time)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i+1, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i);
         CommonOps.addEquals(dAlphaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateDDAlphaPrimeOnCMPSegment(DenseMatrix64F ddAlphaPrime, YoPolynomial cmpPolynomial, double time)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i+2, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i);
         CommonOps.addEquals(ddAlphaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }

   private void calculateBetaPrimeOnCMPSegmentX(DenseMatrix64F betaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialX();
      double timeTrajectory = cmpPolynomialX.getXFinal();
      
      calculateBetaPrimeOnCMPSegment(betaPrime, cmpPolynomialX, time, timeTrajectory);
   }
   
   private void calculateBetaPrimeOnCMPSegmentY(DenseMatrix64F betaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();
      double timeTrajectory = cmpPolynomialY.getXFinal();
      
      calculateBetaPrimeOnCMPSegment(betaPrime, cmpPolynomialY, time, timeTrajectory);
   }
   
   private void calculateBetaPrimeOnCMPSegmentZ(DenseMatrix64F betaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();  
      double timeTrajectory = cmpPolynomialZ.getXFinal();
      
      calculateBetaPrimeOnCMPSegment(betaPrime, cmpPolynomialZ, time, timeTrajectory);
   }
   
   private void calculateBetaPrimeOnCMPSegment(DenseMatrix64F betaPrime, YoPolynomial cmpPolynomial, double time, double timeTrajectory)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i) * Math.exp(omega0.getDoubleValue()*(time-timeTrajectory));
         CommonOps.addEquals(betaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateDBetaPrimeOnCMPSegment(DenseMatrix64F dBetaPrime, YoPolynomial cmpPolynomial, double time, double timeTrajectory)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i-1) * Math.exp(omega0.getDoubleValue()*(time-timeTrajectory));
         CommonOps.addEquals(dBetaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateDDBetaPrimeOnCMPSegment(DenseMatrix64F ddBetaPrime, YoPolynomial cmpPolynomial, double time, double timeTrajectory)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVector.zero();
      
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      tPowersDerivativeVectorTranspose.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
         
         double scalar = Math.pow(1.0/omega0.getDoubleValue(), i-2) * Math.exp(omega0.getDoubleValue()*(time-timeTrajectory));
         CommonOps.addEquals(ddBetaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateGammaPrimeOnCMPSegmentX(DenseMatrix64F gammaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialX = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();
      double timeTrajectory = cmpPolynomialX.getXFinal();
      
      calculateGammaPrimeOnCMPSegment(gammaPrime, time, timeTrajectory);     
   }
   
   private void calculateGammaPrimeOnCMPSegmentY(DenseMatrix64F gammaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialY = cmpPolynomialTrajectories.get(segment).getYoPolynomialY();
      double timeTrajectory = cmpPolynomialY.getXFinal();
      
      calculateGammaPrimeOnCMPSegment(gammaPrime, time, timeTrajectory);    
   }
      
   private void calculateGammaPrimeOnCMPSegmentZ(DenseMatrix64F gammaPrime, int segment, double time)
   {
      YoPolynomial cmpPolynomialZ = cmpPolynomialTrajectories.get(segment).getYoPolynomialZ();
      double timeTrajectory = cmpPolynomialZ.getXFinal();   
      
      calculateGammaPrimeOnCMPSegment(gammaPrime, time, timeTrajectory);
   }

   void calculateGammaPrimeOnCMPSegment(DenseMatrix64F gammaPrime, double time, double timeTrajectory)
   {
      gammaPrime.reshape(1, 1);
      gammaPrime.zero();
      
      double [] gamaPrimeValue = {Math.exp(omega0.getDoubleValue() * (time - timeTrajectory))};
      gammaPrime.setData(gamaPrimeValue);
   }
   
   void calculateDGammaPrimeOnCMPSegment(DenseMatrix64F dGammaPrime, double time, double timeTrajectory)
   {
      dGammaPrime.reshape(1, 1);
      dGammaPrime.zero();
      
      double [] dGamaPrimeValue = {Math.pow(omega0.getDoubleValue(), 1.0)*Math.exp(omega0.getDoubleValue() * (time - timeTrajectory))};
      dGammaPrime.setData(dGamaPrimeValue);
   }
   
   void calculateDDGammaPrimeOnCMPSegment(DenseMatrix64F ddGammaPrime, double time, double timeTrajectory)
   {
      ddGammaPrime.reshape(1, 1);
      ddGammaPrime.zero();
      
      double [] ddGamaPrimeValue = {Math.pow(omega0.getDoubleValue(), 2.0)*Math.exp(omega0.getDoubleValue() * (time - timeTrajectory))};
      ddGammaPrime.setData(ddGamaPrimeValue);
   } 
      
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.set(icpPositionDesiredOutput);
   }
   
   public void getVelocity(FrameVector velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredOutput);
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.set(icpAccelerationDesiredOutput);
   }

   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void showVisualization()
   {
      
   }

   public void hideVisualization()
   {
      
   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }
}
