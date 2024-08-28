#include <TCanvas.h>
#include <TGraph2DErrors.h>
#include <TRandom.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
void flock_data()
{
  std::ifstream infile("pos.txt");

  if (!infile) {
    std::cerr << "Error opening file!" << std::endl;
    return;
  }
  /*
  std::vector<double> pos_0;
  std::vector<double> pos_1;
  std::vector<double> err_0;
  std::vector<double> err_1;
  std::vector<double> time;

  std::string line;
  while (std::getline(infile, line)) {
    if (line[0] == '#')
      continue; // Skip comment lines

    double p_0, e_0, p_1, e_1, t;
    std::istringstream iss(line);
    if (!(iss >> p_0 >> e_0 >> p_1 >> e_1 >> t)) {
      std::cerr << "Error reading line: " << line << std::endl;
      continue; // Skip any line that doesn't have the correct format
    }

    time.push_back(t);
    pos_0.push_back(p_0);
    pos_1.push_back(p_1);
    err_1.push_back(e_1);
    err_0.push_back(e_0);
  }
  std::cout << "alcuni dati " << *pos_0.data()
            << " dimensione dopo: " << time.size() << "\n";
  infile.close();

  int nPoints = time.size();
  for (const auto& value : pos_0)
    std::cout << value << " ";
  std::cout << std::endl;

  for (const auto& value : pos_1)
    std::cout << value << " ";
  std::cout << std::endl;

  // Repeat for other vectors

  TGraph2DErrors* graph =
      new TGraph2DErrors(nPoints, pos_0.data(), pos_1.data(), time.data(),
                         err_0.data(), err_1.data(), nullptr);

  graph->SetTitle("Variable over Time;Time (s);Variable");
  //graph->SetMarkerStyle(kFullCircle);
  //graph->SetMarkerColor(kBlue);
  graph->SetFillColor(29);
  graph->SetMarkerSize(0.8);
  graph->SetMarkerStyle(20);
  graph->SetMarkerColor(kRed);
  graph->SetLineColor(kBlue - 3);
  graph->SetLineWidth(2);
  TCanvas* c1 = new TCanvas("c1", "Plot of Variable over Time", 800, 600);
  graph->Draw("err p0");

  c1->SaveAs("plotFromFile.png");*/

  auto c     = new TCanvas("c", "TGraph2DErrors example", 0, 0, 600, 600);
  Double_t P = 6.;
  Int_t np;

  Double_t *rx = 0, *ry = 0, *rz = 0;
  Double_t *ex = 0, *ey = 0, *ez = 0;

  std::string line;
  std::getline(infile, line);
  std::istringstream iss(line);
  iss >> np;
  rx    = new Double_t[np];
  ry    = new Double_t[np];
  rz    = new Double_t[np];
  ex    = new Double_t[np];
  ey    = new Double_t[np];
  Int_t i = 0;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    iss >> rx[i] >> ex[i] >> ry[i] >> ey[i] >> rz[i];
    i++;
  }

  /*for (Int_t N = 0; N < np; N++) {
    rx[N] = 2 * P * (r->Rndm(N)) - P;
    ry[N] = 2 * P * (r->Rndm(N)) - P;
    rz[N] = rx[N] * rx[N] - ry[N] * ry[N];
    rx[N] = 10. + rx[N];
    ry[N] = 10. + ry[N];
    rz[N] = 40. + rz[N];
    ex[N] = r->Rndm(N);
    ey[N] = r->Rndm(N);
    ez[N] = 10 * r->Rndm(N);
  }*/

  auto g = new TGraph2DErrors(np, rx, ry, rz, ex, ey, ez);
  g->SetTitle("TGraph2D with error bars: option \"ERR\"");
  g->SetFillColor(29);
  g->SetMarkerSize(0.8);
  g->SetMarkerStyle(20);
  g->SetMarkerColor(kRed);
  g->SetLineColor(kBlue - 3);
  g->SetLineWidth(2);
  // gPad->SetLogy(1);
  g->Draw("err p1");
  //g->Draw("L SAME");
}