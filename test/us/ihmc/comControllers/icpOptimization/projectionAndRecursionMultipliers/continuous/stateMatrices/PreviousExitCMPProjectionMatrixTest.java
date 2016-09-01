package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class PreviousExitCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      PreviousExitCMPProjectionMatrix entryCMPProjectionMatrix = new PreviousExitCMPProjectionMatrix(omega, doubleSupportSplitRatio);

      Assert.assertEquals("", 4, entryCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, entryCMPProjectionMatrix.numCols);
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

      PreviousExitCMPProjectionMatrix entryCMPProjectionMatrix = new PreviousExitCMPProjectionMatrix(omega, doubleSupportSplitRatio);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();

         String name = "splitRatio = " + splitRatio + ", doubleSupportDuration = " + doubleSupportDuration;
         boolean isInTransfer = false;

         double initialDoubleSupport = splitRatio * doubleSupportDuration;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, isInTransfer);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         isInTransfer = true;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, isInTransfer);
         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - Math.exp(-omega0 * initialDoubleSupport));
         shouldBe.set(1, 0, -omega0 * Math.exp(-omega0 * initialDoubleSupport));
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

      }
   }
}
