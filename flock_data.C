#include <TCanvas.h>
#include <TGraph2DErrors.h>
#include <TGraphErrors.h>
#include <TROOT.h>
#include <TRandom.h>
#include <TStyle.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
void pos_data()
{
  std::ifstream infile("pos.txt");

  if (!infile) {
    std::cerr << "file cannot be opened!" << std::endl;
    return;
  }

  auto canvas =
      new TCanvas("canvas", "Center of mass of the flock as it varies in time",
                  0, 0, 1000, 600);
  Int_t points_number;

  Double_t *pos_x = 0, *pos_y = 0, *time = 0;
  Double_t *sigma_pos_x = 0, *sigma_pos_y = 0;

  std::string line;
  std::getline(infile, line);
  std::istringstream iss(line);
  iss >> points_number;
  pos_x       = new Double_t[points_number];
  pos_y       = new Double_t[points_number];
  time        = new Double_t[points_number];
  sigma_pos_x = new Double_t[points_number];
  sigma_pos_y = new Double_t[points_number];
  Int_t i     = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    iss >> pos_x[i] >> sigma_pos_x[i] >> pos_y[i] >> sigma_pos_y[i] >> time[i];
    i++;
  }
  // first graph: Drawing the 2D graph of the center of mass as a function of
  // time
  auto graph2D = new TGraph2DErrors(points_number, pos_x, time, pos_y,
                                    sigma_pos_x, nullptr, sigma_pos_y);
  graph2D->SetTitle("Center of mass of the flock as it varies in time;x "
                    "[arb_units];time [arb_units];y [arb_units]");
  graph2D->SetFillColor(1);
  graph2D->SetMarkerSize(0.8);
  graph2D->SetMarkerStyle(26);
  graph2D->SetMarkerColor(kRed);
  graph2D->SetLineColor(kBlack);
  graph2D->SetLineWidth(1);
  graph2D->SetLineStyle(1);
  graph2D->Draw("err p0");
  // canvas->SaveAs("Center of mass 2D plot.png");

  auto canvas_proj = new TCanvas("canvas_proj", "projection of the coordinates",
                                 0, 0, 1000, 1200);
  canvas_proj->Divide(1, 2);
  // Second graph: showing the projection on the x_axis
  auto graph_x_proj =
      new TGraphErrors(points_number, time, pos_x, nullptr, sigma_pos_x);
  graph_x_proj->SetTitle("x-coordinate of the center of mass as a function of "
                         "time;time [arb_units];x [arb_units]");
  graph_x_proj->SetFillColor(1);
  graph_x_proj->SetMarkerSize(0.8);
  graph_x_proj->SetMarkerStyle(26);
  graph_x_proj->SetMarkerColor(kRed);
  graph_x_proj->SetLineColor(kBlack);
  graph_x_proj->SetLineWidth(1);
  graph_x_proj->SetLineStyle(1);
  canvas_proj->cd(1);
  graph_x_proj->Draw("AP");
  // Third graph : showing the projection on the y_axis
  auto graph_y_proj =
      new TGraphErrors(points_number, time, pos_y, nullptr, sigma_pos_y);
  graph_y_proj->SetTitle("y-coordinate of the center of mass as a function of "
                         "time;time [arb_units];y [arb_units]");
  graph_y_proj->SetFillColor(1);
  graph_y_proj->SetMarkerSize(0.8);
  graph_y_proj->SetMarkerStyle(26);
  graph_y_proj->SetMarkerColor(kRed);
  graph_y_proj->SetLineColor(kBlack);
  graph_y_proj->SetLineWidth(1);
  graph_y_proj->SetLineStyle(1);
  canvas_proj->cd(2);
  graph_y_proj->Draw("AP");
  // canvas_proj->SaveAs("x_y_coordinate_plot.png");

  // Drawing the 2D graph of the distribution of the components of velocities
  // with time
}
void vel_data()
{
  std::ifstream vel_infile("vel.txt");

  if (!vel_infile) {
    std::cerr << "file cannot be opened!" << std::endl;
    return;
  }

  auto canvas_vel = new TCanvas(
      "canvas_vel",
      "mean components of velocities in the flock as it varies in time", 0, 0,
      1000, 600);

  Double_t *vel_x = 0, *vel_y = 0, *time = 0;
  Double_t *sigma_vel_x = 0, *sigma_vel_y = 0;
  Int_t points_number;
  std::string line;
  std::getline(vel_infile, line);
  std::istringstream iss(line);
  iss >> points_number;
  vel_x       = new Double_t[points_number];
  vel_y       = new Double_t[points_number];
  sigma_vel_x = new Double_t[points_number];
  sigma_vel_y = new Double_t[points_number];
  time        = new Double_t[points_number];
  Int_t i     = 0;
  while (std::getline(vel_infile, line)) {
    std::istringstream iss(line);
    iss >> vel_x[i] >> sigma_vel_x[i] >> vel_y[i] >> sigma_vel_y[i] >> time[i];
    i++;
  }

  // fourth graph: Drawing the 2D graph of the center of mass as a function of
  // time
  auto graph2D_vel = new TGraph2DErrors(points_number, vel_x, time, vel_y,
                                        sigma_vel_x, nullptr, sigma_vel_y);
  graph2D_vel->SetTitle(
      "Mean components of the velocity as they vary in time;vel_x "
      "[arb_units];time [arb_units];vel_y [arb_units]");
  graph2D_vel->SetFillColor(1);
  graph2D_vel->SetMarkerSize(0.8);
  graph2D_vel->SetMarkerStyle(26);
  graph2D_vel->SetMarkerColor(kRed);
  graph2D_vel->SetLineColor(kBlack);
  graph2D_vel->SetLineWidth(1);
  graph2D_vel->SetLineStyle(1);
  canvas_vel->cd();
  graph2D_vel->Draw("err p1");
  canvas_vel->SaveAs("Velocities 2D graph.png");
  // Drawing the projections

  auto canvas_proj = new TCanvas(
      "canvas_proj", "projection of the components of the velocities", 0, 0,
      1000, 1200);
  canvas_proj->Divide(1, 2);
  // Second graph: showing the projection on the x_axis
  auto graph_x_proj =
      new TGraphErrors(points_number, time, vel_x, nullptr, sigma_vel_x);
  graph_x_proj->SetTitle("x-coordinate of the mean velocity as a function of "
                         "time;time [arb_units];x [arb_units]");
  graph_x_proj->SetFillColor(1);
  graph_x_proj->SetMarkerSize(0.8);
  graph_x_proj->SetMarkerStyle(26);
  graph_x_proj->SetMarkerColor(kRed);
  graph_x_proj->SetLineColor(kBlack);
  graph_x_proj->SetLineWidth(1);
  graph_x_proj->SetLineStyle(1);
  canvas_proj->cd(1);
  graph_x_proj->Draw("AP");
  // Third graph : showing the projection on the y_axis
  auto graph_y_proj =
      new TGraphErrors(points_number, time, vel_y, nullptr, sigma_vel_y);
  graph_y_proj->SetTitle("y-coordinate of the mean velocity as a function of "
                         "time;time [arb_units];y [arb_units]");
  graph_y_proj->SetFillColor(1);
  graph_y_proj->SetMarkerSize(0.8);
  graph_y_proj->SetMarkerStyle(26);
  graph_y_proj->SetMarkerColor(kRed);
  graph_y_proj->SetLineColor(kBlack);
  graph_y_proj->SetLineWidth(1);
  graph_y_proj->SetLineStyle(1);
  canvas_proj->cd(2);
  graph_y_proj->Draw("AP");
  canvas_proj->SaveAs("x_y_velocities_plot.png");
}

void mods_data()
{
  std::ifstream pos_infile("pos_mod.txt");
  std::ifstream vel_infile("vel_mod.txt");
  if (!pos_infile || !vel_infile) {
    std::cerr << "At least one file cannot be opened!" << std::endl;
    return;
  }
  auto canvas_mods = new TCanvas(
      "canvas_mods", "Means of the magnitude of pos and vel", 0, 0, 1000, 1200);
  canvas_mods->Divide(1, 2);
  auto graph_pos_mod = new TGraphErrors("pos_mod.txt", "%lg %lg %lg");

  graph_pos_mod->SetTitle("mean magnitude of position a function of "
                          "time;time [arb_units];x [arb_units]");
  graph_pos_mod->SetFillColor(1);
  graph_pos_mod->SetMarkerSize(0.8);
  graph_pos_mod->SetMarkerStyle(26);
  graph_pos_mod->SetMarkerColor(kRed);
  graph_pos_mod->SetLineColor(kBlack);
  graph_pos_mod->SetLineWidth(1);
  graph_pos_mod->SetLineStyle(1);
  canvas_mods->cd(1);
  graph_pos_mod->Draw("AP");
  // Drawing the mean magnitude of velocities
  auto graph_vel_mod = new TGraphErrors("vel_mod.txt", "%lg %lg %lg");

  graph_vel_mod->SetTitle("mean magnitude of velocities a function of "
                          "time;time [arb_units];x [arb_units]");
  graph_vel_mod->SetFillColor(1);
  graph_vel_mod->SetMarkerSize(0.8);
  graph_vel_mod->SetMarkerStyle(26);
  graph_vel_mod->SetMarkerColor(kRed);
  graph_vel_mod->SetLineColor(kBlack);
  graph_vel_mod->SetLineWidth(1);
  graph_vel_mod->SetLineStyle(1);
  canvas_mods->cd(2);
  graph_vel_mod->Draw("AP");
  //Drawing the mean distance over time
  auto canvas_distance = new TCanvas(
      "canvas_distance", "Mean distance", 0, 0, 1000, 600);
  auto graph_distance = new TGraphErrors("distance.txt", "%lg %lg %lg");

  graph_distance->SetTitle("mean distance as a function of "
                          "time;time [arb_units];x [arb_units]");
  graph_distance->SetFillColor(1);
  graph_distance->SetMarkerSize(0.8);
  graph_distance->SetMarkerStyle(26);
  graph_distance->SetMarkerColor(kRed);
  graph_distance->SetLineColor(kBlack);
  graph_distance->SetLineWidth(1);
  graph_distance->SetLineStyle(1);
  canvas_distance->cd();
  graph_distance->Draw("AP");

}