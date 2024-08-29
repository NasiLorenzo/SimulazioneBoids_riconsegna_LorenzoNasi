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

void graph_2d(std::string file, const char* title, const char* save_as)
{
  std::ifstream infile(file);

  if (!infile) {
    std::cerr << "file cannot be opened!" << std::endl;
    return;
  }
  Int_t points_number;
  Double_t *_x = 0, *_y = 0, *time = 0;
  Double_t *sigma_x = 0, *sigma_y = 0;
  std::string line;
  std::getline(infile, line);
  std::istringstream iss(line);
  iss >> points_number;
  _x      = new Double_t[points_number];
  _y      = new Double_t[points_number];
  time    = new Double_t[points_number];
  sigma_x = new Double_t[points_number];
  sigma_y = new Double_t[points_number];
  Int_t i = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    iss >> _x[i] >> sigma_x[i] >> _y[i] >> sigma_y[i] >> time[i];
    i++;
  }
  // first graph: Drawing the 2D graph of the center of mass as a function of
  // time
  auto graph2D = new TGraph2DErrors(points_number, _x, time, _y, sigma_x,
                                    nullptr, sigma_y);
  graph2D->SetTitle(title);
  graph2D->SetFillColor(1);
  graph2D->SetMarkerSize(0.8);
  graph2D->SetMarkerStyle(26);
  graph2D->SetMarkerColor(kRed);
  graph2D->SetLineColor(kBlack);
  graph2D->SetLineWidth(1);
  graph2D->SetLineStyle(3);
  graph2D->Draw("err p0");
  graph2D->SaveAs(save_as);
}
void graph_1d_proj(TCanvas* canvas, std::string filename, const char* title,
                   Int_t canvas_number)
{
  std::ifstream infile(filename);

  if (!infile) {
    std::cerr << "file cannot be opened!" << std::endl;
    return;
  }
  Int_t points_number;
  Double_t *_coord = 0, *time = 0;
  Double_t* sigma_coord = 0;
  Double_t skip;
  std::string line;
  std::getline(infile, line);
  std::istringstream iss(line);
  iss >> points_number;
  _coord      = new Double_t[points_number];
  time        = new Double_t[points_number];
  sigma_coord = new Double_t[points_number];
  Int_t i     = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    if (canvas_number == 1) {
      if (!(iss >> _coord[i] >> sigma_coord[i] >> skip >> skip >> time[i])) {
        throw std::runtime_error{"Not enough columns!"};
      }
    } else {
      if (!(iss >> skip >> skip >> _coord[i] >> sigma_coord[i] >> time[i])) {
        throw std::runtime_error{"Not enough columns!"};
      }
    }

    i++;
  }
  auto graph_proj =
      new TGraphErrors(points_number, time, _coord, nullptr, sigma_coord);
  graph_proj->SetTitle(title);
  graph_proj->SetFillColor(1);
  graph_proj->SetMarkerSize(0.8);
  graph_proj->SetMarkerStyle(26);
  graph_proj->SetMarkerColor(kRed);
  graph_proj->SetLineColor(kBlack);
  graph_proj->SetLineWidth(1);
  graph_proj->SetLineStyle(1);
  canvas->cd(canvas_number);
  graph_proj->Draw("AP");
}

void graph_1d(TCanvas* canvas, const char* filename, const char* title,
                   Int_t canvas_number)
{
  std::ifstream infile(filename);

  if (!infile) {
    std::cerr << "file cannot be opened!" << std::endl;
    return;
  }
  auto graph_1d =
      new TGraphErrors(filename,"%lg %lg %lg","");
  graph_1d->SetTitle(title);
  graph_1d->SetFillColor(1);
  graph_1d->SetMarkerSize(0.8);
  graph_1d->SetMarkerStyle(26);
  graph_1d->SetMarkerColor(kRed);
  graph_1d->SetLineColor(kBlack);
  graph_1d->SetLineWidth(1);
  graph_1d->SetLineStyle(1);
  canvas->cd(canvas_number);
  graph_1d->Draw("AP");
}



void pos_data()
{
  // first graph: Drawing the 2D graph of the center of mass as a function of
  auto canvas =
      new TCanvas("canvas", "Center of mass of the flock as it varies in time",
                  0, 0, 1000, 600);
  canvas->cd();
  graph_2d("pos.txt",
           "Center of mass of the flock as it varies in time;x "
           "[arb_units];time [arb_units];y [arb_units]",
           "Center_of_mass_2D_plot.png");

  auto canvas_proj = new TCanvas("canvas_proj", "projection of the coordinates",
                                 0, 0, 1000, 1200);
  canvas_proj->Divide(1, 2);
  // Second graph: showing the projection on the x_axis
  graph_1d_proj(canvas_proj, "pos.txt",
                "x-coordinate of the center of mass as a function of "
                "time;time [arb_units];x [arb_units]",
                1);
  // Third graph : showing the projection on the y_axis
  graph_1d_proj(canvas_proj, "pos.txt",
                "y-coordinate of the center of mass as a function of "
                "time;time [arb_units];y [arb_units]",
                2);
  canvas_proj->SaveAs("x_y_projections.png");

  /*auto graph_x_proj =
      new TGraphErrors(points_number, time, pos_x, nullptr, sigma_pos_x);
  graph_x_proj->SetTitle("x-coordinate of the center of mass as a function of
  " "time;time [arb_units];x [arb_units]"); graph_x_proj->SetFillColor(1);
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
  graph_y_proj->SetTitle("y-coordinate of the center of mass as a function of
  " "time;time [arb_units];y [arb_units]"); graph_y_proj->SetFillColor(1);
  graph_y_proj->SetMarkerSize(0.8);
  graph_y_proj->SetMarkerStyle(26);
  graph_y_proj->SetMarkerColor(kRed);
  graph_y_proj->SetLineColor(kBlack);
  graph_y_proj->SetLineWidth(1);
  graph_y_proj->SetLineStyle(1);
  canvas_proj->cd(2);
  graph_y_proj->Draw("AP");*/
  // canvas_proj->SaveAs("x_y_coordinate_plot.png");*/

  // Drawing the 2D graph of the distribution of the components of velocities
  // with time
}
void vel_data()
{
  auto canvas_vel = new TCanvas(
      "canvas_vel", "Center of mass of the flock as it varies in time", 0, 0,
      1000, 600);
  canvas_vel->cd();
  // fourth graph: Drawing the 2D graph of the center of mass as a function of
  // time
  graph_2d("vel.txt",
           "Mean components of the velocity as they vary in time;vel_x "
           "[arb_units];time [arb_units];vel_y [arb_units]",
           "Velocities_2D_graph.png");
  auto canvas_vel_proj =
      new TCanvas("canvas_vel_proj", "projection of the velocity components", 0,
                  0, 1000, 1200);
  canvas_vel_proj->Divide(1, 2);
  canvas_vel_proj->cd();
  // Fifth graph: showing the projection on the x_axis
  graph_1d_proj(canvas_vel_proj, "vel.txt",
                "x-coordinate of the mean velocity as a function of "
                "time;time [arb_units];x [arb_units]",
                1);
  // Sixth graph : showing the projection on the y_axis
  graph_1d_proj(canvas_vel_proj, "vel.txt",
                "y-coordinate of the mean velocity as a function of "
                "time;time [arb_units];y [arb_units]",
                2);
  canvas_vel_proj->SaveAs("x_y_vel_projections.png");
  /*std::ifstream vel_infile("vel.txt");

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
    iss >> vel_x[i] >> sigma_vel_x[i] >> vel_y[i] >> sigma_vel_y[i] >>
  time[i]; i++;
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
  // Drawing the projections*/

  /*auto canvas_proj = new TCanvas(
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
  canvas_proj->SaveAs("x_y_velocities_plot.png");*/
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
  // Drawing the mean distance over time
  auto canvas_distance =
      new TCanvas("canvas_distance", "Mean distance", 0, 0, 1000, 600);
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