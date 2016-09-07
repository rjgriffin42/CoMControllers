package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferStateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class TransferStateEndRecursionMatrixTest
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

      TransferStateEndRecursionMatrix transferStateEndRecursionMatrix = new TransferStateEndRecursionMatrix(omega);

      Assert.assertEquals("", 4, transferStateEndRecursionMatrix.numRows);
      Assert.assertEquals("", 1, transferStateEndRecursionMatrix.numCols);
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

      TransferStateEndRecursionMatrix transferStateEndRecursionMatrix = new TransferStateEndRecursionMatrix(omega);

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

         transferStateEndRecursionMatrix.compute(currentDoubleSupportDuration);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);

         transferStateEndRecursionMatrix.compute(currentDoubleSupportDuration);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * singleSupportDuration));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * singleSupportDuration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);

         double endOfDoubleSupport = (1.0 - splitRatio) * currentDoubleSupportDuration;

         transferStateEndRecursionMatrix.compute(currentDoubleSupportDuration);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);

         double upcomingInitialDoubleSupport = splitRatio * upcomingDoubleSupportDuration;
         double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

         transferStateEndRecursionMatrix.compute(currentDoubleSupportDuration);
         shouldBe.zero();
         double duration = upcomingInitialDoubleSupport + endOfDoubleSupport - stepDuration;
         shouldBe.set(0, 0, Math.exp(omega0 * duration));
         shouldBe.set(1, 0, omega0 * Math.exp(omega0 * duration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);
      }
   }
}
