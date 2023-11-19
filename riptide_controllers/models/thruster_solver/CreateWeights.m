%initialize the thruster pose matrix
%really the matrix B from B * F = Uf
thrusterSolver.thrusterPoses = [transpose(eul2rotm(importedData.robot.thrusters.VUSDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.VUSPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.VUSDir, "XYZ") * [1;0;0])) ;
                                transpose(eul2rotm(importedData.robot.thrusters.VUPDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.VUPPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.VUPDir, "XYZ") * [1;0;0])) ;
                                transpose(eul2rotm(importedData.robot.thrusters.VLSDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.VLSPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.VLSDir, "XYZ") * [1;0;0])) ;
                                transpose(eul2rotm(importedData.robot.thrusters.VLPDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.VLPPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.VLPDir, "XYZ") * [1;0;0])) ;
                                transpose(eul2rotm(importedData.robot.thrusters.HUSDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.HUSPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.HUSDir, "XYZ") * [1;0;0])) ;
                                transpose(eul2rotm(importedData.robot.thrusters.HUPDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.HUPPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.HUPDir, "XYZ") * [1;0;0])) ;
                                transpose(eul2rotm(importedData.robot.thrusters.HLSDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.HLSPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.HLSDir, "XYZ") * [1;0;0])) ;
                                transpose(eul2rotm(importedData.robot.thrusters.HLPDir, "XYZ") * [1;0;0]), transpose(cross(transpose(importedData.robot.thrusters.HLPPos - importedData.robot.com), eul2rotm(importedData.robot.thrusters.HLPDir, "XYZ") * [1;0;0])) ;];

%weights matrix
thrusterSolver.weights.standard = [ 1,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,1;];

%low downwash weights
thrusterSolver.weights.lowDownwash = [  1,0,0,0,0,0,0,0;
                                        0,1,0,0,0,0,0,0;
                                        0,0,1,0,0,0,0,0;
                                        0,0,0,1,0,0,0,0;
                                        0,0,0,0,1,0,0,0;
                                        0,0,0,0,0,1,0,0;
                                        0,0,0,0,0,0,3,0;
                                        0,0,0,0,0,0,0,3;];

%VUS fault
thrusterSolver.weights.VUSFault = [ 9999,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,1;];

%VUP fault
thrusterSolver.weights.VUPFault = [ 1,0,0,0,0,0,0,0;
                                    0,9999,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,1;];

%VLS fault
thrusterSolver.weights.VLSFault = [ 1,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,9999,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,1;];

%VLP fault
thrusterSolver.weights.VLPFault = [ 1,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,999,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,1;];

%HUS fault
thrusterSolver.weights.HUSFault = [ 1,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,9999,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,1;];

%HUP fault
thrusterSolver.weights.HUPFault = [ 1,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,9999,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,1;];

%HLS fault
thrusterSolver.weights.HLSFault = [ 1,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,9999,0;
                                    0,0,0,0,0,0,0,1;];

%HLP fault
thrusterSolver.weights.HLPFault = [ 1,0,0,0,0,0,0,0;
                                    0,1,0,0,0,0,0,0;
                                    0,0,1,0,0,0,0,0;
                                    0,0,0,1,0,0,0,0;
                                    0,0,0,0,1,0,0,0;
                                    0,0,0,0,0,1,0,0;
                                    0,0,0,0,0,0,1,0;
                                    0,0,0,0,0,0,0,9999;];





thrusterSolver.B.standard = inv(thrusterSolver.weights.standard) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.standard) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.lowDownwash = inv(thrusterSolver.weights.lowDownwash) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.lowDownwash) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.VUSFault = inv(thrusterSolver.weights.VUSFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.VUSFault) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.VUPFault = inv(thrusterSolver.weights.VUPFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.VUPFault) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.VLSFault = inv(thrusterSolver.weights.VLSFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.VLSFault) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.VLPFault = inv(thrusterSolver.weights.VLPFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.VLPFault) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.HUSFault = inv(thrusterSolver.weights.HUSFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.HUSFault) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.HUPFault = inv(thrusterSolver.weights.HUPFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.HUPFault) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.HLPFault = inv(thrusterSolver.weights.HLSFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.HLSFault) * (thrusterSolver.thrusterPoses)));
thrusterSolver.B.HLSFault = inv(thrusterSolver.weights.HLPFault) * (thrusterSolver.thrusterPoses) * inv((transpose(thrusterSolver.thrusterPoses) * inv(thrusterSolver.weights.HLPFault) * (thrusterSolver.thrusterPoses)));





