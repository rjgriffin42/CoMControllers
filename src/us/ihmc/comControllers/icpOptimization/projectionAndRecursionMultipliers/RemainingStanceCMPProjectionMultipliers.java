package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public interface RemainingStanceCMPProjectionMultipliers
{
   public void reset();

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry);

   public double getExitMultiplier();

   public double getEntryMultiplier();

   public double getPreviousExitMultiplier();
}
