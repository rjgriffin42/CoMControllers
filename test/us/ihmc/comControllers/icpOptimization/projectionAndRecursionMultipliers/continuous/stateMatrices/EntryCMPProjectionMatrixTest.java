package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.continuous.stateMatrices;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class EntryCMPProjectionMatrixTest
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
      EntryCMPProjectionMatrix entryCMPProjectionMatrix = new EntryCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime);

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
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);

      EntryCMPProjectionMatrix entryCMPProjectionMatrix = new EntryCMPProjectionMatrix(omega, doubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime);

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

         entryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         isInTransfer = false;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         useTwoCMPs = true;
         isInTransfer = true;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(3, 0, -omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         isInTransfer = false;
         double duration = endOfDoubleSupport - timeOnEntry;
         entryCMPProjectionMatrix.compute(doubleSupportDuration, singleSupportDuration, useTwoCMPs, isInTransfer);
         shouldBe.zero();
         shouldBe.set(0, 0, (1.0 - Math.exp(omega0 * duration)));
         shouldBe.set(1, 0, -omega0 * Math.exp(omega0 * duration));
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);
      }
   }
}