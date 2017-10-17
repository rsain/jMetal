package org.uma.jmetal.util.comparator;

import java.io.Serializable;
import java.util.Comparator;
import java.util.List;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.solutionattribute.impl.PreferenceDistance;

public class PreferenceDistanceComparator <S extends Solution<?>> implements Comparator<S>,
    Serializable {
  private PreferenceDistance<S> preferenceDistance;

  public  PreferenceDistanceComparator(PreferenceDistance<S> preferenceDistance){
    this.preferenceDistance = preferenceDistance;
  }
  @Override
  public int compare(S solution1, S solution2) {
    int result ;
    if (solution1 == null) {
      if (solution2 == null) {
        result = 0;
      } else {
        result = 1 ;
      }
    } else if (solution2 == null) {
      result = -1;
    } else {
      double distance1 = Double.MIN_VALUE ;
      double distance2 = Double.MIN_VALUE ;

      if (preferenceDistance.getAttribute(solution1) != null) {
        distance1 = (double) preferenceDistance.getAttribute(solution1);
      }

      if (preferenceDistance.getAttribute(solution2) != null) {
        distance2 = (double) preferenceDistance.getAttribute(solution2);
      }

      if (distance1 > distance2) {
        result = -1;
      } else  if (distance1 < distance2) {
        result = 1;
      } else {
        result = 0;
      }
    }

    return result ;
  }
}
