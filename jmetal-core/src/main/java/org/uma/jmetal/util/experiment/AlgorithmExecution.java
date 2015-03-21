//  AlgorithmExecution.java
//
//  Authors:
//       Antonio J. Nebro <antonio@lcc.uma.es>
//
//  Copyright (c) 2014 Antonio J. Nebro
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

package org.uma.jmetal.util.experiment;

import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;

import java.io.File;

/**
 * Created by Antonio J. Nebro on 18/07/14.
 */
public class AlgorithmExecution implements Experiment {
  private String paretoSetFileName ;
  private String paretoFrontFileName ;
  private ExperimentConfiguration configuration ;

  /** Constructor */
  private AlgorithmExecution(Builder builder) {
    paretoFrontFileName = builder.paretoFrontFileName ;
    paretoSetFileName = builder.paretoSetFileName ;
    configuration = builder.configuration ;
  }

  /* Getters */
  public String getParetoSetFileName() {
    return paretoSetFileName;
  }

  public String getParetoFrontFileName() {
    return paretoFrontFileName;
  }

  public ExperimentConfiguration getExperimentData() {
    return configuration;
  }

  /** Builder class */
  public static class Builder {
    private final ExperimentConfiguration configuration ;
    private String paretoSetFileName ;
    private String paretoFrontFileName ;

    /** Builder class */
    public Builder(ExperimentConfiguration configuration) {
      this.configuration = configuration ;
      paretoFrontFileName = "FUN" ;
      paretoSetFileName = "VAR" ;
    }

    public Builder setParetoFrontFileName(String fileName) {
      paretoFrontFileName = fileName ;

      return this ;
    }

    public Builder setParetoSetFileName(String fileName) {
      paretoSetFileName = fileName ;

      return this ;
    }

    public AlgorithmExecution build() {
      return new AlgorithmExecution(this) ;
    }
  }

  @Override
  public void run() {
    JMetalLogger.logger.info("ExperimentExecution: Running algorithms");
    if (experimentDirectoryDoesNotExist()) {
       createExperimentDirectory() ;
    }
/*
    parallelExecutor.start(this);

    for (String algorithm : experimentData.getAlgorithmNameList()) {
      for (String problem : experimentData.getProblemList()) {
        for (int i = 0; i < experimentData.getIndependentRuns(); i++) {
          JMetalLogger.logger.info(
            "Adding task. Algorithm:  " + algorithm + " Problem: " + problem + " Run: " + i);
          parallelExecutor.addTask(new Object[] {algorithm, problem, i, experimentData});
        }
      }
    }

    parallelExecutor.parallelExecution();
    parallelExecutor.stop();
    */
  }

  private boolean experimentDirectoryDoesNotExist() {
    boolean result;
    File experimentDirectory;

    experimentDirectory = new File(configuration.getExperimentBaseDirectory());
    if (experimentDirectory.exists() && experimentDirectory.isDirectory()) {
      result = false;
    } else {
      result = true;
    }

    return result;
  }

  private void createExperimentDirectory() {
    File experimentDirectory;
    experimentDirectory = new File(configuration.getExperimentBaseDirectory());

    if (experimentDirectory.exists()) {
      experimentDirectory.delete() ;
    }

    boolean result ;
    result = new File(configuration.getExperimentBaseDirectory()).mkdirs() ;
    if (!result) {
      throw new JMetalException("Error creating experiment directory: " +
        configuration.getExperimentBaseDirectory()) ;
    }
  }
}
