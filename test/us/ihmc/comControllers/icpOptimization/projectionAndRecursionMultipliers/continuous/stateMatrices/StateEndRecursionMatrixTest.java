package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class StateEndRecursionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);

      StateEndRecursionMatrix stateEndRecursionMatrix = new StateEndRecursionMatrix(omega, doubleSupportSplitRatio, exitCMPRatio);

      Assert.assertEquals("", 4, stateEndRecursionMatrix.numRows);
      Assert.assertEquals("", 1, stateEndRecursionMatrix.numCols);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;



      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPRatio = new DoubleYoVariable("exitCMPRatio", registry);

      StateEndRecursionMatrix stateEndRecursionMatrix = new StateEndRecursionMatrix(omega, doubleSupportSplitRatio, exitCMPRatio);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();

         double startOfSplineTime = 0.1 * singleSupportDuration;
         double endOfSplineTime = 0.8 * singleSupportDuration;

         String name = "splitRatio = " + splitRatio + ",\n doubleSupportDuration = " + currentDoubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         stateEndRecursionMatrix.computeInTransfer(currentDoubleSupportDuration);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, stateEndRecursionMatrix, epsilon);

         stateEndRecursionMatrix.computeInSingleSupport(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, startOfSplineTime, endOfSplineTime);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * singleSupportDuration));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * singleSupportDuration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, stateEndRecursionMatrix, epsilon);

         double endOfDoubleSupport = (1.0 - splitRatio) * currentDoubleSupportDuration;

         stateEndRecursionMatrix.computeInTransfer(currentDoubleSupportDuration);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, stateEndRecursionMatrix, epsilon);

         double upcomingInitialDoubleSupport = splitRatio * upcomingDoubleSupportDuration;
         double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

         stateEndRecursionMatrix.computeInSingleSupport(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDuration, startOfSplineTime, endOfSplineTime);
         shouldBe.zero();
         double duration = upcomingInitialDoubleSupport + endOfDoubleSupport - stepDuration;
         shouldBe.set(0, 0, Math.exp(omega0 * duration));
         shouldBe.set(1, 0, omega0 * Math.exp(omega0 * duration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, stateEndRecursionMatrix, epsilon);
      }
   }
}
