package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class SmoothCapturePointAdjustmentToolbox
{
   private static final int defaultSize = 100;

   private double generalizedGammaPrimeSegment1;
   private final DenseMatrix64F generalizedAlphaPrimeRowSegment1 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeRowSegment1 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedAlphaBetaPrimeRowSegment1 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F polynomialCoefficientVectorAdjustmentSegment1 = new DenseMatrix64F(defaultSize, 1);

   private double generalizedGammaPrimeSegment2;
   private final DenseMatrix64F generalizedAlphaPrimeRowSegment2 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeRowSegment2 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedAlphaBetaPrimeRowSegment2 = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F polynomialCoefficientVectorAdjustmentSegment2 = new DenseMatrix64F(defaultSize, 1);

   private final DenseMatrix64F polynomialCoefficientCombinedVectorAdjustment = new DenseMatrix64F(defaultSize, 1);

   private final DenseMatrix64F boundaryConditionMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F boundaryConditionMatrixInverse = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F boundaryConditionVector = new DenseMatrix64F(defaultSize, 1);

   private final LinearSolver<DenseMatrix64F> pseudoInverseSolver = new SolvePseudoInverseSvd();

   private List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList = new ArrayList<FrameTuple3D<?, ?>>();

   private final SmoothCapturePointToolbox icpToolbox;

   public SmoothCapturePointAdjustmentToolbox(SmoothCapturePointToolbox smoothCapturePointToolbox)
   {
      this.icpToolbox = smoothCapturePointToolbox;

      icpQuantityInitialConditionList.add(new FramePoint3D());
      while (icpQuantityInitialConditionList.size() < defaultSize)
      {
         icpQuantityInitialConditionList.add(new FrameVector3D());
      }
   }

   public void setICPInitialConditions(double localTime, List<FramePoint3D> exitCornerPointsFromCoPs, List<FrameTrajectory3D> copPolynomials3D,
                                       int currentSwingSegment, double omega0)
   {
      if (currentSwingSegment < 0)
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(0);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            copPolynomial3D.getDerivative(i, localTime, icpQuantityInitialCondition);
         }
      }
      else
      {
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(currentSwingSegment);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3D<?, ?> icpQuantityInitialCondition = icpQuantityInitialConditionList.get(i);

            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, localTime, i, copPolynomial3D,
                                                                            exitCornerPointsFromCoPs.get(currentSwingSegment), icpQuantityInitialCondition);
         }
      }
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing(List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack,
                                                            List<FrameTrajectory3D> copPolynomials3D, double omega0)
   {
      adjustDesiredTrajectoriesForInitialSmoothing3D(omega0, copPolynomials3D, icpQuantityInitialConditionList, entryCornerPointsToPack,
                                                     exitCornerPointsToPack);
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing3D(double omega0, List<FrameTrajectory3D> copPolynomials3D,
                                                              List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList,
                                                              List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack)
   {
      FrameTrajectory3D cmpPolynomial3DSegment1 = copPolynomials3D.get(0);
      FrameTrajectory3D cmpPolynomial3DSegment2 = copPolynomials3D.get(1);
      FramePoint3D icpPositionFinalSegment2 = exitCornerPointsToPack.get(1);
      for (Direction direction : Direction.values)
      {
         Trajectory cmpPolynomialSegment1 = cmpPolynomial3DSegment1.getTrajectory(direction);
         Trajectory cmpPolynomialSegment2 = cmpPolynomial3DSegment2.getTrajectory(direction);

         double icpPositionFinalSegment2Scalar = icpPositionFinalSegment2.getElement(direction.getIndex());

         int numberOfCoefficients = cmpPolynomialSegment1.getNumberOfCoefficients();
         int numberOfConstrainedDerivatives = numberOfCoefficients / 2;

         initializeMatrices1D(numberOfCoefficients, numberOfConstrainedDerivatives);

         populateBoundaryConditionMatrices1D(omega0, direction, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1,
                                             cmpPolynomialSegment2, icpQuantityInitialConditionList, icpPositionFinalSegment2Scalar);
         computeAdjustedPolynomialCoefficientVectors1D(numberOfCoefficients);
         adjustCMPPolynomials(cmpPolynomialSegment1, cmpPolynomialSegment2);
      }
      icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsToPack, exitCornerPointsToPack, copPolynomials3D, omega0);
   }

   private void adjustCMPPolynomials(Trajectory cmpPolynomialSegment1, Trajectory cmpPolynomialSegment2)
   {
      cmpPolynomialSegment1.setDirectly(polynomialCoefficientVectorAdjustmentSegment1);
      cmpPolynomialSegment2.setDirectly(polynomialCoefficientVectorAdjustmentSegment2);
   }

   private void populateBoundaryConditionMatrices1D(double omega0, Direction direction, int numberOfCoefficients, int numberOfConstrainedDerivatives,
                                                    Trajectory cmpPolynomialSegment1, Trajectory cmpPolynomialSegment2,
                                                    List<FrameTuple3D<?, ?>> icpQuantityInitialConditionList, double icpPositionFinalSegment2Scalar)
   {
      calculateGeneralizedICPMatricesOnCMPSegment2(omega0, cmpPolynomialSegment2);

      // TODO: check whether division always integer
      for (int i = 0; i < numberOfConstrainedDerivatives; i++)
      {
         double icpQuantityInitialConditionScalar = icpQuantityInitialConditionList.get(i).getElement(direction.getIndex());
         calculateGeneralizedICPMatricesOnCMPSegment1(omega0, i, cmpPolynomialSegment1);
         setGeneralizedBoundaryConstraints(i, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1, cmpPolynomialSegment2,
                                           icpQuantityInitialConditionScalar, icpPositionFinalSegment2Scalar);
      }
   }

   private static void setGeneralizedBoundaryConstraintICP0(DenseMatrix64F boundaryConditionVectorToPack, DenseMatrix64F boundaryConditionMatrixToPack,
                                                     int order, int numberOfCoefficients, double icpQuantityInitialConditionScalar,
                                                     double icpPositionFinalSegment2,
                                                     DenseMatrix64F generalizedAlphaBetaPrimeRowSegment1, double generalizedGammaPrimeMatrixSegment1,
                                                     DenseMatrix64F generalizedAlphaBetaPrimeRowSegment2, double generalizedGammaPrimeMatrixSegment2)
   {
      double generalizedBoundaryConditionValue = icpQuantityInitialConditionScalar - generalizedGammaPrimeMatrixSegment1 * generalizedGammaPrimeMatrixSegment2 * icpPositionFinalSegment2;
      boundaryConditionVectorToPack.set(order, generalizedBoundaryConditionValue);

      MatrixTools.setMatrixBlock(boundaryConditionMatrixToPack, order, 0, generalizedAlphaBetaPrimeRowSegment1, 0, 0, generalizedAlphaBetaPrimeRowSegment1.numRows,
                                 generalizedAlphaBetaPrimeRowSegment1.numCols, 1.0);

      MatrixTools.setMatrixBlock(boundaryConditionMatrixToPack, order, numberOfCoefficients, generalizedAlphaBetaPrimeRowSegment2, 0, 0, generalizedAlphaBetaPrimeRowSegment2.numRows,
                                 generalizedAlphaBetaPrimeRowSegment2.numCols, generalizedGammaPrimeMatrixSegment1);
   }

   private static void setGeneralizedBoundaryConstraintCMP0(DenseMatrix64F boundaryConditionVectorToPack, DenseMatrix64F boundaryConditionMatrixToPack,
                                                     int order, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1)
   {
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();

      boundaryConditionVectorToPack.set(order + numberOfConstrainedDerivatives, cmpPolynomialSegment1.getDerivative(order, tInitial1));

      DenseMatrix64F xPowersDerivativeVector = cmpPolynomialSegment1.getXPowersDerivativeVector(order, tInitial1);
      MatrixTools.setMatrixBlock(boundaryConditionMatrixToPack, order + numberOfConstrainedDerivatives, 0, xPowersDerivativeVector, 0, 0,
                                 xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, 1.0);
   }

   private static void setGeneralizedBoundaryConstraintCMP1(DenseMatrix64F boundaryConditionVectorToPack, DenseMatrix64F boundaryConditionMatrixToPack,
                                                     int order, int numberOfCoefficients, int numberOfConstrainedDerivatives,
                                                     Trajectory cmpPolynomialSegment1, Trajectory cmpPolynomialSegment2)
   {
      double tFinal1 = cmpPolynomialSegment1.getFinalTime();
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();

      boundaryConditionVectorToPack.set(order + 2 * numberOfConstrainedDerivatives, 0.0);

      DenseMatrix64F xPowersDerivativeVector = cmpPolynomialSegment1.getXPowersDerivativeVector(order, tFinal1);
      MatrixTools.setMatrixBlock(boundaryConditionMatrixToPack, order + 2 * numberOfConstrainedDerivatives, 0, xPowersDerivativeVector, 0, 0,
                                 xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, -1.0);

      xPowersDerivativeVector = cmpPolynomialSegment2.getXPowersDerivativeVector(order, tInitial2);
      MatrixTools.setMatrixBlock(boundaryConditionMatrixToPack, order + 2 * numberOfConstrainedDerivatives, numberOfCoefficients, xPowersDerivativeVector, 0, 0,
                                 xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, 1.0);
   }

   private static void setGeneralizedBoundaryConstraintCMP2(DenseMatrix64F boundaryConditionVectorToPack, DenseMatrix64F boundaryConditionMatrixToPack,
                                                     int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment2)
   {
      double tFinal2 = cmpPolynomialSegment2.getFinalTime();

      boundaryConditionVectorToPack.set(order + 3 * numberOfConstrainedDerivatives, cmpPolynomialSegment2.getDerivative(order, tFinal2));

      DenseMatrix64F xPowersDerivativeVector = cmpPolynomialSegment2.getXPowersDerivativeVector(order, tFinal2);
      MatrixTools.setMatrixBlock(boundaryConditionMatrixToPack, order + 3 * numberOfConstrainedDerivatives, numberOfCoefficients, xPowersDerivativeVector,
                                 0, 0, xPowersDerivativeVector.numRows, xPowersDerivativeVector.numCols, 1.0);
   }

   private void computeAdjustedPolynomialCoefficientVectors1D(int numberOfCoefficients)
   {
      // Uses the Moore-Penrose pseudo-inverse to counter bad conditioning of boundaryConditionMatrix
      pseudoInverseSolver.setA(boundaryConditionMatrix);
      pseudoInverseSolver.solve(boundaryConditionVector, polynomialCoefficientCombinedVectorAdjustment);

      MatrixTools.setMatrixBlock(polynomialCoefficientVectorAdjustmentSegment1, 0, 0, polynomialCoefficientCombinedVectorAdjustment, 0, 0, numberOfCoefficients, 1, 1.0);
      MatrixTools.setMatrixBlock(polynomialCoefficientVectorAdjustmentSegment2, 0, 0, polynomialCoefficientCombinedVectorAdjustment, numberOfCoefficients, 0, numberOfCoefficients, 1, 1.0);
   }

   private void calculateGeneralizedICPMatricesOnCMPSegment2(double omega0, Trajectory cmpPolynomialSegment2)
   {
      double tInitial2 = cmpPolynomialSegment2.getInitialTime();
      generalizedGammaPrimeSegment2 = SmoothCapturePointToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial2, 0, cmpPolynomialSegment2, generalizedAlphaPrimeRowSegment2, generalizedBetaPrimeRowSegment2,
                                                                 generalizedAlphaBetaPrimeRowSegment2);
   }

   private void calculateGeneralizedICPMatricesOnCMPSegment1(double omega0, int derivativeOrder, Trajectory cmpPolynomialSegment1)
   {
      double tInitial1 = cmpPolynomialSegment1.getInitialTime();
      generalizedGammaPrimeSegment1 = SmoothCapturePointToolbox.calculateGeneralizedMatricesPrimeOnCMPSegment1D(omega0, tInitial1, derivativeOrder, cmpPolynomialSegment1, generalizedAlphaPrimeRowSegment1,
                                                                 generalizedBetaPrimeRowSegment1, generalizedAlphaBetaPrimeRowSegment1);
   }

   private void setGeneralizedBoundaryConstraints(int order, int numberOfCoefficients, int numberOfConstrainedDerivatives, Trajectory cmpPolynomialSegment1,
                                                  Trajectory cmpPolynomialSegment2, double icpQuantityInitialConditionScalar,
                                                  double icpPositionFinalSegment2Scalar)
   {
      setGeneralizedBoundaryConstraintICP0(boundaryConditionVector, boundaryConditionMatrix, order, numberOfCoefficients, icpQuantityInitialConditionScalar, icpPositionFinalSegment2Scalar,
                                           generalizedAlphaBetaPrimeRowSegment1, generalizedGammaPrimeSegment1,
                                           generalizedAlphaBetaPrimeRowSegment2, generalizedGammaPrimeSegment2);
      setGeneralizedBoundaryConstraintCMP0(boundaryConditionVector, boundaryConditionMatrix, order, numberOfConstrainedDerivatives, cmpPolynomialSegment1);
      setGeneralizedBoundaryConstraintCMP1(boundaryConditionVector, boundaryConditionMatrix, order, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment1, cmpPolynomialSegment2);
      setGeneralizedBoundaryConstraintCMP2(boundaryConditionVector, boundaryConditionMatrix, order, numberOfCoefficients, numberOfConstrainedDerivatives, cmpPolynomialSegment2);
   }

   private void initializeMatrices1D(int numberOfCoefficients, int numberOfConstrainedDerivatives)
   {
      boundaryConditionMatrix.reshape(4 * numberOfConstrainedDerivatives, 2 * numberOfCoefficients);
      boundaryConditionMatrixInverse.reshape(2 * numberOfCoefficients, 4 * numberOfConstrainedDerivatives);
      boundaryConditionVector.reshape(4 * numberOfConstrainedDerivatives, 1);

      polynomialCoefficientCombinedVectorAdjustment.reshape(2 * numberOfCoefficients, 1);

      polynomialCoefficientVectorAdjustmentSegment1.reshape(numberOfCoefficients, 1);
      polynomialCoefficientVectorAdjustmentSegment2.reshape(numberOfCoefficients, 1);

      generalizedAlphaPrimeRowSegment1.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeRowSegment1.reshape(1, numberOfCoefficients);
      generalizedAlphaBetaPrimeRowSegment1.reshape(1, numberOfCoefficients);

      generalizedAlphaPrimeRowSegment2.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeRowSegment2.reshape(1, numberOfCoefficients);
      generalizedAlphaBetaPrimeRowSegment2.reshape(1, numberOfCoefficients);
   }
}
