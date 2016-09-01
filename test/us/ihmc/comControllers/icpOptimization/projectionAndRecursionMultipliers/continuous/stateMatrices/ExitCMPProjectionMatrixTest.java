package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class ExitCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      ExitCMPProjectionMatrix exitCMPProjectionMatrix = new ExitCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime);

      Assert.assertEquals("", 4, exitCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, exitCMPProjectionMatrix.numCols);
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
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);

      ExitCMPProjectionMatrix exitCMPProjectionMatrix = new ExitCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();

         String name = "splitRatio = " + splitRatio + ", exitRatio = " + exitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;
         boolean useTwoCMPs = false;
         boolean isInTransfer = true;

         double initialDoubleSupport = splitRatio * doubleSupportDuration;
         double endOfDoubleSupport = (1.0 - splitRatio) * doubleSupportDuration;
         double timeOnEntry = (1.0 - exitRatio) * (singleSupportDuration + doubleSupportDuration);
         double timeOnExit = exitRatio * (singleSupportDuration + doubleSupportDuration);

         exitCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(3, 0, -omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, exitCMPProjectionMatrix, epsilon);

         isInTransfer = false;

         exitCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         shouldBe.set(0, 0, (1.0 - Math.exp(-omega0 * singleSupportDuration)));
         shouldBe.set(1, 0, -omega0 * Math.exp(-omega0 * singleSupportDuration));
         shouldBe.set(3, 0, -omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, exitCMPProjectionMatrix, epsilon);

         useTwoCMPs = true;
         isInTransfer = true;

         exitCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, exitCMPProjectionMatrix, epsilon);

         isInTransfer = false;
         double duration = endOfDoubleSupport - timeOnEntry;
         exitCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(omega0 * duration) * (1.0 - Math.exp(omega0 * (initialDoubleSupport - timeOnExit))));
         shouldBe.set(1, 0, omega0 * Math.exp(omega0 * duration) * (1.0 - Math.exp(omega0 * (initialDoubleSupport - timeOnExit))));
         shouldBe.set(3, 0, -omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, exitCMPProjectionMatrix, epsilon);
      }
   }
}
