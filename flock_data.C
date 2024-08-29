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

void graph_2d(std::string file, const char* title, const char* save_as, TCanvas* canvas)
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
  graph2D->SetFillColor(kRed);
  graph2D->SetMarkerSize(0.8);
  graph2D->SetMarkerStyle(26);
  graph2D->SetMarkerColor(kRed);
  graph2D->SetLineColor(kBlack);
  graph2D->SetLineWidth(1);
  graph2D->SetLineStyle(3);
  graph2D->Draw("err p1");
  canvas->SaveAs(save_as);
}
void graph_1d_proj_3d(TCanvas* canvas, std::string filename, const char* title,
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
      if (!(iss >> _coord[i] >> sigma_coord[i] >> skip >> skip >> skip >> skip
            >> time[i])) {
        throw std::runtime_error{"Not enough columns!"};
      }
    } else {
      if (canvas_number == 2) {
        if (!(iss >> skip >> skip >> _coord[i] >> sigma_coord[i] >> skip >> skip
              >> time[i])) {
          throw std::runtime_error{"Not enough columns!"};
        }
      } else {
        if (canvas_number == 3) {
          if (!(iss >> skip >> skip >> skip >> skip >> _coord[i]
                >> sigma_coord[i] >> time[i])) {
            throw std::runtime_error{"Not enough columns!"};
          }
        }
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
      if (canvas_number == 2) {
        if (!(iss >> skip >> skip >> _coord[i] >> sigma_coord[i] >> time[i])) {
          throw std::runtime_error{"Not enough columns!"};
        }
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
  auto graph_1d = new TGraphErrors(filename, "%lg %lg %lg", "");
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
           "Center_of_mass_2D_plot.png",canvas);

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
           "Velocities_2D_graph.png",canvas_vel);
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
}

void mods_data()
{
  auto canvas_mods = new TCanvas(
      "canvas_mods", "Means of the magnitude of pos and vel", 0, 0, 1000, 1200);
  canvas_mods->Divide(1, 2);
  // Seventh graph: drawing the mean magnitude of position as a function of
  // time
  graph_1d(canvas_mods, "pos_mod.txt",
           "mean distance from the origin as a function of "
           "time;time [arb_units];x [arb_units]",
           1);

  // Eight graph: drawing the mean magnitude of position as a function of time
  graph_1d(canvas_mods, "vel_mod.txt",
           "mean speed a function of "
           "time;time [arb_units];x [arb_units]",
           2);
  canvas_mods->SaveAs("Mean_distance_from_origin_mean_speed.png");

  // Ninth grap: drawing the mean distance over time
  auto canvas_distance =
      new TCanvas("canvas_distance", "Mean distance", 0, 0, 1000, 600);
  graph_1d(canvas_distance, "distance.txt",
           "mean distance as a function of "
           "time;time [arb_units];x [arb_units]",
           1);
  canvas_distance->SaveAs("Mean_distance.png");
}
// Graphs for the 3d version
void pos_data_3d()
{
  auto canvas_proj = new TCanvas("canvas_proj", "projection of the coordinates",
                                 0, 0, 1000, 1200);
  canvas_proj->Divide(2, 2);
  // First graph: showing the projection on the x_axis
  graph_1d_proj_3d(canvas_proj, "pos3d.txt",
                "x-coordinate of the center of mass as a function of "
                "time;time [arb_units];x [arb_units]",
                1);
  // Second graph : showing the projection on the y_axis
  graph_1d_proj_3d(canvas_proj, "pos3d.txt",
                "y-coordinate of the center of mass as a function of "
                "time;time [arb_units];y [arb_units]",
                2);
  // Second graph : showing the projection on the z_axis
  graph_1d_proj_3d(canvas_proj, "pos3d.txt",
                "z-coordinate of the center of mass as a function of "
                "time;time [arb_units];z [arb_units]",
                3);
  canvas_proj->SaveAs("x_y_z_projections.png");
}
void vel_data_3d()
{
  auto canvas_vel_proj =
      new TCanvas("canvas_vel_proj", "projection of the velocity components", 0,
                  0, 1000, 1200);
  canvas_vel_proj->Divide(2, 2);
  canvas_vel_proj->cd();
  // Fifth graph: showing the projection on the x_axis
  graph_1d_proj_3d(canvas_vel_proj, "vel3d.txt",
                "x-coordinate of the mean velocity as a function of "
                "time;time [arb_units];x [arb_units]",
                1);
  // Sixth graph : showing the projection on the y_axis
  graph_1d_proj_3d(canvas_vel_proj, "vel3d.txt",
                "y-coordinate of the mean velocity as a function of "
                "time;time [arb_units];y [arb_units]",
                2);
  // Sixth graph : showing the projection on the y_axis
  graph_1d_proj_3d(canvas_vel_proj, "vel3d.txt",
                "z-coordinate of the mean velocity as a function of "
                "time;time [arb_units];z [arb_units]",
                3);
  canvas_vel_proj->SaveAs("x_y_z_vel_projections.png");
}

void mods_data_3d()
{
  auto canvas_mods = new TCanvas(
      "canvas_mods", "Means of the magnitude of pos and vel", 0, 0, 1000, 1200);
  canvas_mods->Divide(1, 2);
  // Seventh graph: drawing the mean magnitude of position as a function of
  // time
  graph_1d(canvas_mods, "pos_mod3d.txt",
           "mean distance from the origin as a function of "
           "time;time [arb_units];x [arb_units]",
           1);

  // Eight graph: drawing the mean magnitude of position as a function of time
  graph_1d(canvas_mods, "vel_mod3d.txt",
           "mean speed a function of "
           "time;time [arb_units];x [arb_units]",
           2);
  canvas_mods->SaveAs("Mean_distance_from_origin_mean_speed_3d.png");

  // Ninth grap: drawing the mean distance over time
  auto canvas_distance =
      new TCanvas("canvas_distance", "Mean distance", 0, 0, 1000, 600);
  graph_1d(canvas_distance, "distance3d.txt",
           "mean distance as a function of "
           "time;time [arb_units];x [arb_units]",
           1);
  canvas_distance->SaveAs("Mean_distance_3d.png");
}