#include "doublevec.hpp"
#include "sfmlinterface.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <cmath>

float cameraX      = -180.4f;
float cameraY      = -110.f;
float cameraZ      = -262.4f;
float cameraAngleY = 0.0f;

void drawGridBorders(float pixel_x, float pixel_y, float pixel_z)
{
  glBegin(GL_LINES);

  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(pixel_x, 0.0f, 0.0f);
  glVertex3f(pixel_x, 0.0f, 0.0f);
  glVertex3f(pixel_x, pixel_y, 0.0f);
  glVertex3f(pixel_x, pixel_y, 0.0f);
  glVertex3f(0.0f, pixel_y, 0.0f);
  glVertex3f(0.0f, pixel_y, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);

  glVertex3f(0.0f, 0.0f, pixel_z);
  glVertex3f(pixel_x, 0.0f, pixel_z);
  glVertex3f(pixel_x, 0.0f, pixel_z);
  glVertex3f(pixel_x, pixel_y, pixel_z);
  glVertex3f(pixel_x, pixel_y, pixel_z);
  glVertex3f(0.0f, pixel_y, pixel_z);
  glVertex3f(0.0f, pixel_y, pixel_z);
  glVertex3f(0.0f, 0.0f, pixel_z);

  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, pixel_z);
  glVertex3f(pixel_x, 0.0f, 0.0f);
  glVertex3f(pixel_x, 0.0f, pixel_z);
  glVertex3f(pixel_x, pixel_y, 0.0f);
  glVertex3f(pixel_x, pixel_y, pixel_z);
  glVertex3f(0.0f, pixel_y, 0.0f);
  glVertex3f(0.0f, pixel_y, pixel_z);

  glEnd();
}

void loadPerspectiveMatrix(float fov, float aspect, float zNear, float zFar)
{
  float f     = 1.0f / std::tan(fov / 2.0f);
  float depth = zFar - zNear;

  float perspective[16] = {f / aspect,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           f,
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f,
                           -(zFar + zNear) / depth,
                           -1.0f,
                           0.0f,
                           0.0f,
                           -2.0f * zFar * zNear / depth,
                           0.0f};

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(perspective);
}

float boidX      = 0.0f;
float boidY      = 0.0f;
float boidZ      = -5.0f;

void drawBoid(float boidx, float boidy, float boidz)
{
  glPushMatrix();
  glTranslatef(boidx, boidy, boidz);

  glBegin(GL_TRIANGLES);
  glColor3f(1.0f, 0.0f, 0.0f);

  glVertex3f(0.0f, 0.0f, 0.2f);    // Top vertex of the cone
  glVertex3f(-0.2f, -0.2f, -0.2f); // Bottom-left vertex
  glVertex3f(0.2f, -0.2f, -0.2f);  // Bottom-right vertex

  glVertex3f(0.0f, 0.0f, 0.2f);   // Top vertex of the cone
  glVertex3f(0.2f, -0.2f, -0.2f); // Bottom-right vertex
  glVertex3f(0.0f, 0.2f, -0.2f);  // Bottom-middle vertex

  glVertex3f(0.0f, 0.0f, 0.2f);    // Top vertex of the cone
  glVertex3f(0.0f, 0.2f, -0.2f);   // Bottom-middle vertex
  glVertex3f(-0.2f, -0.2f, -0.2f); // Bottom-left vertex

  glVertex3f(-0.2f, -0.2f, -0.2f); // Bottom-left vertex
  glVertex3f(0.2f, -0.2f, -0.2f);  // Bottom-right vertex
  glVertex3f(0.0f, 0.2f, -0.2f);   // Bottom-middle vertex

  glEnd();

  glPopMatrix(); // Restore the previous matrix state
}

void handleKeyboardInput(sf::Keyboard::Key key, bool isPressed)
{
  float moveSpeed     = 8.f;
  float rotationSpeed = 0.02f;

  if (key == sf::Keyboard::W) {
    cameraZ += moveSpeed /* cos(cameraAngleY)*/;
  }
  if (key == sf::Keyboard::S) {
    cameraZ -= moveSpeed /** cos(cameraAngleY)*/;
  }
  if (key == sf::Keyboard::A) {
    cameraX += moveSpeed;
  }
  if (key == sf::Keyboard::D) {
    cameraX -= moveSpeed;
  }
  if (key == sf::Keyboard::Down) {
    cameraY += moveSpeed;
  }
  if (key == sf::Keyboard::Up) {
    cameraY -= moveSpeed;
  }
  if (key == sf::Keyboard::Left) {
    cameraAngleY -= rotationSpeed;
  }
  if (key == sf::Keyboard::Right) {
    cameraAngleY += rotationSpeed;
  }
}

void drawGrid(float pixel_x, float pixel_y, float pixel_z, float view_range)
{
  int num_x = static_cast<int>(pixel_x / view_range);
  int num_y = static_cast<int>(pixel_y / view_range);
  int num_z = static_cast<int>(pixel_z / view_range);

  glBegin(GL_LINES);

  for (int i = 0; i <= num_y; i++) {
    for (int j = 0; j <= num_z; j++) {
      glVertex3f(0.0f, i * view_range, j * view_range);
      glVertex3f(pixel_x, i * view_range, j * view_range);
    }
  }

  for (int i = 0; i <= num_x; i++) {
    for (int j = 0; j <= num_z; j++) {
      glVertex3f(i * view_range, 0.0f, j * view_range);
      glVertex3f(i * view_range, pixel_y, j * view_range);
    }
  }

  for (int i = 0; i <= num_x; i++) {
    for (int j = 0; j <= num_y; j++) {
      glVertex3f(i * view_range, j * view_range, 0.0f);
      glVertex3f(i * view_range, j * view_range, pixel_z);
    }
  }

  glEnd();
}

int main(int argc, char* argv[])
{
  std::random_device r;
  std::default_random_engine eng{r()};
  boids::ParamList params{"parametersfml3d.txt"};
  boids::check_parallelism(argc, argv, params);
  boids::Flock flock{eng, params};
  sf::RenderWindow window(sf::VideoMode(1920, 1080), "3D with OpenGL in SFML");
  window.setActive(true); // Activate the OpenGL context
  flock.update(params);
  glEnable(GL_DEPTH_TEST);
  sf::Clock clock;
  const sf::Time frameTime = sf::seconds(params.deltaT);
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }

      else if (event.type == sf::Event::KeyPressed) {
        handleKeyboardInput(event.key.code, true);
      } else if (event.type == sf::Event::KeyReleased) {
        handleKeyboardInput(event.key.code, false);
      }
    }
    clock.restart();
    glDisable(GL_CULL_FACE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    flock.update(params);
    // Load perspective matrix
    loadPerspectiveMatrix(90.f * M_PI / 180.0f, 1920.0f / 1080.0f, 1.0f,
                          500.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glRotatef(cameraAngleY * (180.0f / M_PI), 0.0f, 1.0f, 0.0f);

    glTranslatef(cameraX, cameraY, cameraZ);
 
    drawGrid(params.pixel[0], params.pixel[1], params.pixel[2],params.view_range);
    for (int i = 0; i < params.size; i++) {
      /*std::cout<<"boid disegnato at: "<<flock.set()[i].pos()[0]<<",
         "<<flock.set()[i].pos()[1]<<", "<< flock.set()[i].pos()[2]<<"\n";*/
      drawBoid(flock.set()[i].pos()[0], flock.set()[i].pos()[1],
               flock.set()[i].pos()[2]);
      /*std::cout << "posizione finale: " << cameraX << cameraY << cameraZ
                << "\n";*/
    }

    window.display();
    sf::sleep(frameTime - clock.getElapsedTime());
  }

  return 0;
}
