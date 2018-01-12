package org.uma.jmetal.utility;

import java.util.ArrayList;
import java.util.List;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;

public class GenerateWeights {
  public static void main(String[] args){
    List<List<Double>> aux = weightInitial(5);
    int x=0;
  }

  private static List<List<Double>> generateWeight(int obj, int size,double lower,double upper){
    List<List<Double>> result =weightInitial(obj);
    List<List<Double>> weightAux= new ArrayList<>();
    JMetalRandom jMetalRandom = JMetalRandom.getInstance();
    for (int i = 0; i < 5000 ; i++) {
      List<Double> aux= new ArrayList<>();
      for (int j = 0; j < obj; j++) {
        aux.add(jMetalRandom.nextDouble(lower,upper));
      }
    }
    for (int i = obj; i < size ; i++) {

    }
    return result;
  }

private static List<List<Double>> weightInitial(int obj){
  List<List<Double>> result = new ArrayList<>();
  for (int i=0;i<obj;i++ ){
    List<Double> aux= new ArrayList<>();
    for (int j = 0; j < obj ; j++) {
      aux.add(0.0);
    }
    aux.set(i,1.0);
    result.add(aux);
  }
  return result;
}
}
