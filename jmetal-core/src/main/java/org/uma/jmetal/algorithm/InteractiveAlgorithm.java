package org.uma.jmetal.algorithm;

import java.util.List;
import org.uma.jmetal.util.referencePoint.ReferencePoint;

public interface InteractiveAlgorithm<S,R> extends Algorithm<R>{
  public void updateInterestPoint(List<ReferencePoint> newReferencePoints);
}
