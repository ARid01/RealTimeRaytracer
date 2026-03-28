#define WIN32_LEAN_AND_MEAN         //Exclude rarely used APIs, speeding up compilation and avoiding namespace pollution
#include <windows.h>                //Windows API for creating windows, handling messages and system calls
#include <gl/GL.h>                  //OpenGL core functions for rendering
#include <stdint.h>                 //Standard integer types and math functions
#include <math.h>                   //-
#include <cmath>                    //-
#include <vector>                   //C++ STL containers for dynamic arrays and strings
#include <string>                   //-
#include <chrono>                   //For timing and potentially threading (althought thread is not in use yet)
#include <thread>
#pragma comment(lib, "opengl32.lib")//Links OpenGL library automatically when compiling with MSVC

//-------- Configuration --------
const int WIDTH = 800;              //Window Width in Pixels
const int HEIGHT = 600;             //Window Height in Pixels
const int LOW_WIDTH = 100;          //Raytrace Resolution Width
const int LOW_HEIGHT = 75;          //Raytrace Resolution Height

//-------- Utilities -----------
const float EPS = 1e-4f;            //Small epsilon value used for floating point offsets to avoid self-intersections or precision errors
const float MAX_DIST = 30.0f;       //Max distance along a ray to march before giving up (ie, far plane)
const int MAX_STEPS = 128;          //How many discrete steps the sphere-tracing algorithm will take per ray
const int MAX_BOUNCES = 6;          //Allows multiple bounces/refractions/reflections for the ray, simulating realistic optics

//-------- Simple math vector --------
struct Vec3 {
    float x, y, z;                                                                      //Custom 3D Vector struct for handling rays, locations, directions, etc
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float a, float b, float c) : x(a), y(b), z(c) {}
    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }
};
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }                                            //Scalar dot product between two vectors
inline Vec3 normalize(const Vec3& v) { float l = sqrtf(dot(v, v)); return v / (l > 0 ? l : 1.0f); }                                     //Scales a vector to unit length
inline Vec3 cross(const Vec3& a, const Vec3& b) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }   //Returns a vector perpendicular to a and b (right hand rule)
inline float length(const Vec3& a) { return sqrtf((a.x * a.x) + (a.y * a.y) + (a.z * a.z)); }                                           //Returns magnitude (length) of vector
inline Vec3 reflect(const Vec3& I, const Vec3& N) { return I - N * 2.0f * dot(I, N); }                                                  //Reflects the incident vector (I) about the normal vector (N)

//Ray
struct Ray { Vec3 o, d; Ray(const Vec3& o_, const Vec3& d_) :o(o_), d(d_) {} };         //Simple Ray structure represented in 3D (o = origin, d = direction)

//-------- Camera ------------
class Camera {
public:
    Vec3 position;      //World position
    Vec3 forward;       //Direction camera faces
    Vec3 up;            //Up vector for camera orientation
    float fov;          //Field of view in degrees
    float aspect;       //Aspect ratio (width/height) of viewport

    //Orbiting data
    Vec3 target;        //The point the camera orbits around
    float radius;       //Distance from position to target
    float azimuth;      //Horizontal angle (left, right rotation around target
    float elevation;    //Vertical angle (up, down)

    Camera(const Vec3& pos, const Vec3& lookAt, float fovDeg, float aspectRatio)            //Initializes position, target, fov and aspect
        : position(pos), fov(fovDeg), aspect(aspectRatio),                                  //Calculates initial forward vector as normalized(lookAt - pos)
          target(lookAt), radius(length(lookAt - pos)), azimuth(0.0f), elevation(0.0f)      //Sets default up vector pointing straight up
    {                                                                                       //Computes the spherical coordinates of the camera relative to the target
        forward = normalize(lookAt - pos);
        up = Vec3(0, 1, 0);

        Vec3 offset = pos - target;
        azimuth = atan2f(offset.z, offset.x);                                               //angle in the horizontal plane using atan2f    
        elevation = asinf(offset.y / radius);                                               //angle in the vertical plane using inverse sin (arcsin)
    }

    void updateOrbit(float deltaAzimuth, float deltaElevation) {                            //Changes azimuth and elevation by given deltas
        azimuth += deltaAzimuth;
        elevation += deltaElevation;

        const float maxElev = 1.55f; // ~89 degrees                                         //Clamps elevation to prevent flipping of camera
        if (elevation > maxElev) elevation = maxElev;
        if (elevation < -maxElev) elevation = -maxElev;

        position.x = target.x + radius * cosf(elevation) * cosf(azimuth);                   //Converts spherical coordinates back to Cartesian
        position.y = target.y + radius * sinf(elevation);
        position.z = target.z + radius * cosf(elevation) * sinf(azimuth);

        forward = normalize(target - position);                                             //Updates forward to point to target
    }

    void zoom(float delta) {                                                                //Changes radius by delta
        radius += delta;
        if (radius < 0.1f) radius = 0.1f;                                                   //Clamps minimum radius to 0.1 to prevent going into target
        updateOrbit(0.0f, 0.0f);                                                            //Calls update orbit with 0 deltas to update position and forward
    }

    Ray getRay(int px, int py, int width, int height) const {                                       //Converts pixel coordinates (px, py) into a ray from the cam into scene
        float ndc_x = (px + 0.5f) / (float)width;                                                   //Computes normalized device coordinates (NDC) in [0,1] space
        float ndc_y = (py + 0.5f) / (float)height;
        float screen_x = (2.0f * ndc_x - 1.0f) * aspect * tanf(fov * 0.5f * 3.14159265f / 180.0f);  //Maps NDC to screen space coordinates with perspective projection using FOV and aspect ratio
        float screen_y = (1.0f - 2.0f * ndc_y) * tanf(fov * 0.5f * 3.14159265f / 180.0f);

        //Build a simple camera basis
        Vec3 right = normalize(cross(forward, up));                                                 //Builds an orthonormal camera basis
        Vec3 trueUp = cross(right, forward);                                                        //Right vector via cross product of forward and up
        Vec3 dir = normalize(forward + right * screen_x + trueUp * screen_y);                       //TrueUp vector recomputed as perpendicular to right and forward
                                                                                                    //Computes direct of ray as a normalized vector pointing through pixel (px, py) on the image plane
        return Ray(position, dir);                                                                  //Returns a ray with origin at position and the computed direction
    }

    void move(const Vec3& delta) {                                                                  //Moves camera position by a delta vector (unused currently)
        position = position + delta;
    }

    void lookAt(const Vec3& target) {                                                               //Updates forward direction to look at a new point (unused currently)
        forward = normalize(target - position);
    }
};

// Sphere
struct Sphere { Vec3 c; float r; Sphere(const Vec3& c_, float r_) :c(c_), r(r_) {} };               //Defines a sphere by its center (c) and radius (r)

float sphereSDF(const Vec3& p, const Sphere& sp) {                                                  //Signed Distance Function (SDF) for a sphere
    return length(p - sp.c) - sp.r;                                                                 //Calculates the Euclidean distance from a point (p) to a sphere center (sp.c) and subtracts radius
}                                                                                                   //--Positive outside the sphere, zero on the surface, negative inside

float sceneSDF(const Vec3& p, const std::vector<Sphere>& scene) {           //Returns the shortest distance from (p) to any object in the scene
    float minDist = 1e20f;                                                  //Loops over all speheres, computes their SDF, and finds the minimum distance
    for (const Sphere& sp : scene) {                                        //This forms the core for sphere-tracing
        float dist = sphereSDF(p, sp);
        if (dist < minDist) minDist = dist;
    }
    return minDist;
}

Vec3 getWaterNormal(const Vec3& p, float time) {
    float A = 0.05f;
    float k = 3.0f;
    float omega = 5.0f;

    float dx = A * k * cosf(k * p.x + omega * time);
    float dz = A * k * cosf(k * p.z + omega * time);

    Vec3 n = normalize(Vec3(-dx, -1.0f, -dz));
    return n;
}

bool refract_dir(const Vec3& I, const Vec3& N, float eta, Vec3& outT) {     //Calculates refraction direction (outT) of an incident ray (I) hitting a surface with normal (N) and index ratio (eta=n1/n2)
    float cosi = fmaxf(-1.0f, fminf(1.0f, -dot(N, I)));                     //Uses Snell's law vector form
    float k = 1.0f - eta * eta * (1.0f - cosi * cosi);                      //cosi is clamped cosine of angle between incident ray and normal
    if (k < 0.0f) {                                                         //k determines if refraction is possible; if negative, total internal reflection occurs
        return false; // TIR
    }
    outT = I * eta + N * (eta * cosi - sqrtf(k));
    outT = normalize(outT);
    return true;                                                            //Returns true if refraction is valid, false if Total Internal Reflection
}

void shade_color(unsigned char* dst, const Vec3& pos, const Vec3& normal) {     //Basic Lambertian diffuse shading
    Vec3 L = normalize(Vec3(-1, -1, -1));                                       //L is the normalized light direction (from light to surface) 
    float lam = fmaxf(0.0f, dot(normal, -L));                                   //lam is the cosine of the angle between surface normal and light direction, clamped to [0, 1]
    float baseR = 0.4f + 0.6f * fabsf(normal.x);                                //Base Color depends on the absolute value of each nromal component, giving a color-coded normal effect
    float baseG = 0.4f + 0.6f * fabsf(normal.y);
    float baseB = 0.4f + 0.6f * fabsf(normal.z);
    float r = baseR * (0.1f + 0.9f * lam);                                      //Multiplies by light intensity with ambient 0.1 and diffuse 0.9
    float g = baseG * (0.1f + 0.9f * lam);
    float b = baseB * (0.1f + 0.9f * lam);
    dst[0] = (unsigned char)(fminf(1.0f, r) * 255);                             //Clamps and converts to 0-255 RGB bytes
    dst[1] = (unsigned char)(fminf(1.0f, g) * 255);
    dst[2] = (unsigned char)(fminf(1.0f, b) * 255);
}

void marchRay(unsigned char* dst, Ray ray, const std::vector<Sphere>& scene, float waterLevel, unsigned long int waterFrameCount) {  
                                                                                //This function implements sphere-tracing with refraction and reflection at the water plane (y=0), multiple bounces and water tint
    bool insideWater = (ray.o.y < waterLevel);                                  //dst->pointer to a buffer where the rgb color of the pixel is stored
                                                                                //ray is the ray casting into the scene (which is a vector of spheres)
    for (int bounce = 0; bounce < MAX_BOUNCES; ++bounce) {                      //Starts by checking if we begin the ray under water (default 0.0, may be changed later)
        float t = 0.0f;                                                         //Starts a loop for multiple ray bounces/refractions/reflections
        bool hitSomething = false;                                              //Default 6, so it can simulate complex interactions like reflection/refraction on water and spheres
        Vec3 hitPos, hitNormal;

        for (int step = 0; step < MAX_STEPS && t < MAX_DIST; ++step) {          //Inner loop for ray marching steps, capped by 128 Max Steps and 30 Unit Max Distance
            Vec3 p = ray.o + ray.d * t;                                         //Ray marches forward in increments determined by the signed distance function (SDF)
                                                                                //Calculates the current sample point (p) along the ray (in 3D space)
            float d = sceneSDF(p, scene);                                       //sceneSDF returns minimum distance from p to any object surface (spheres)
                                                                                //This distance (d) tells us how far we can move safely without hitting anything
            bool handledPlaneThisStep = false;
            if (fabsf(ray.d.y) > 1e-8f) {                                       //Check that the ray is not exactly parallel to the plane (fabs(ray.d.y) > small epsilon)
                float tPlane = -ray.o.y / ray.d.y;                              //Calculate the intersection distance with the plane by solving the parametric equation for the ray
                if (tPlane > t + EPS && tPlane < t + d) {                       //ray.o.y + ray,d,y * tPlane = 0 -> solve for tPlane
                    Vec3 planeHit = ray.o + ray.d * tPlane;                             //Check if the plane intersection is in front of the current position (t) but closer than the SDF distance (d)
                    Vec3 normal = getWaterNormal(planeHit, waterFrameCount * 0.01f);
                    if (ray.d.y < waterLevel) normal = -normal;                               //Means that plane is the closest surface on this step
                                                                                        //Calculate the exact hit point on the plane, and determin the surface normal of the plane
                    float n1 = (ray.d.y < 0.0f) ? 1.0f : 1.33f;                         //If ray is going doward, (entering water) normal is upward, otherwise normal is downward
                    float n2 = (ray.d.y < 0.0f) ? 1.33f : 1.0f;                         //Define the refractive indices for air and water, and calculate eta for Snell's law
                    float eta = n1 / n2;

                    Vec3 tdir;                                                          //Call refract_dir function to compute the refracted ray direction
                    if (refract_dir(ray.d, normal, eta, tdir)) {                        //Returns true if the refraction is possible (no Total Internal Reflection)
                        ray.o = planeHit + tdir * EPS * 4.0f;                           //If refraction succeeds: move ray origin slightly beyond the plane (EPS * 4) to avoid precision issues
                        ray.d = tdir;                                                   //Update the ray direction to the refracted direction

                        // Update insideWater flag when crossing plane
                        insideWater = (ray.o.y < waterLevel);                           //Update the insideWater flag to reflect new ray position
                    }
                    else {                                                              //If refraction fails (TIR), reflect the ray instead and update origin and direction accordingly
                        Vec3 rdir = reflect(ray.d, normal);
                        ray.o = planeHit + normalize(rdir) * EPS * 4.0f;
                        ray.d = normalize(rdir);
                    }
                    handledPlaneThisStep = true;                                        //Mark that the plane interaction was handled in this step to skip further marching
                }
            }

            if (handledPlaneThisStep) {                                                     //Exit marching loop to start next bounce/refraction/reflection iteration with updated ray
                break;
            }

            if (d < 1e-3f) {                                                                //If the signed distance is very small (close enough to surface) consider the ray hit
                hitSomething = true;                                                        //Save hit position
                hitPos = p;

                const float h = 1e-3f;                                                      //To estimate the surface normal, sample the SDF slightly offset in x, y, and z directions
                Vec3 nx = Vec3(h, 0, 0);                                                    //h is a small delta for numerical gradient estimation
                Vec3 ny = Vec3(0, h, 0);
                Vec3 nz = Vec3(0, 0, h);
                float dx = sceneSDF(p + nx, scene) - sceneSDF(p - nx, scene);               //Compute finite differences in each direction
                float dy = sceneSDF(p + ny, scene) - sceneSDF(p - ny, scene);               //This approximates the gradient of the distance field (which equals the surface normal direction)
                float dz = sceneSDF(p + nz, scene) - sceneSDF(p - nz, scene);
                hitNormal = normalize(Vec3(dx, dy, dz));                                    //Normalize the gradient vector to get the normal at the surface
                break;                                                                      //Exit the marching loop as we found a hit
            }
            t += d;                                                                         //If no hit yet, increment t by the distance d and continue marching
        }

        if (hitSomething) {                                                                 //If hit, compute base color using shade_color function, which applies lighting based on the surface normal
            unsigned char baseColor[3];                                                     //and directional light
            shade_color(baseColor, hitPos, hitNormal);

            if (insideWater || hitPos.y < waterLevel) {                                     //If the surface is underwater, apply a water tint
                // Apply strong water blue tint here (like we discussed)                    //Convert color components from [0, 255] range to [0,1] floats
                float rf = baseColor[0] / 255.0f;
                float gf = baseColor[1] / 255.0f;
                float bf = baseColor[2] / 255.0f;

                Vec3 waterTint = Vec3(0.0f, 0.3f, 0.8f);                                    //Blend base color with a blueish tint to simulate water coloring
                float darkness = 0.6f;                                                      //Darkness lowers overall brightness

                rf = rf * darkness * 0.3f + waterTint.x * (1.0f - 0.3f);                    //The mix blends 30% base color and 70% water tint
                gf = gf * darkness * 0.3f + waterTint.y * (1.0f - 0.3f);
                bf = bf * darkness * 0.3f + waterTint.z * (1.0f - 0.3f);

                dst[0] = (unsigned char)(fminf(1.0f, rf) * 255);                            //Clamp and convert back into 8-bit RGB
                dst[1] = (unsigned char)(fminf(1.0f, gf) * 255);
                dst[2] = (unsigned char)(fminf(1.0f, bf) * 255);
            }
            else {
                // normal shading
                dst[0] = baseColor[0];                                                      //If not underwater, assign the base shaded color
                dst[1] = baseColor[1];
                dst[2] = baseColor[2];
            }
            return;                                                                         //Return early since the pixel color is decided
        }
    }                                                                                       //End of boucnes loop.  If no hit, continue to background

    // If we got here, no geometry hit, so:                                                 //If no geometry hit after max bounces and marching steps:
    if (insideWater) {                                                                      //If ray inside water, paint a solid deep blue background to simulate underwater
        // Show solid deep blue tint for water background
        dst[0] = 10;
        dst[1] = 40;
        dst[2] = 120;
    }
    else {                                                                                  //Otherwise, paint a sky gradient that depnds on the ray's vertical angle (ray.d.y)
        // Normal sky background gradient                                                   //Interpolates between two sky colors from horizon to zenith
        float tt = 0.5f * (ray.d.y + 1.0f);
        float r = (1.0f - tt) * 0.7f + tt * 0.2f;
        float g = (1.0f - tt) * 0.9f + tt * 0.5f;
        float b = 1.0f;
        dst[0] = (unsigned char)(fminf(1.0f, r) * 255);                                     //Assign final color to dst
        dst[1] = (unsigned char)(fminf(1.0f, g) * 255);
        dst[2] = (unsigned char)(fminf(1.0f, b) * 255);
    }
}

#pragma region OpenGLSetup                                                              
                                                                                            //Global Handles for:
static HDC g_hDC = nullptr;                                                                 //g_hDC: Device Context, required for OpenGL drawing on Win32
static HGLRC g_glRC = nullptr;                                                              //g_glRC: OpenGL Rendering COntext
static GLuint g_tex = 0;                                                                    //g_tex: OpenGL TextureID

void CreateTexture(int w, int h) {                                                          //Creates an OpenGL texture of width w and height h
    if (g_tex) glDeleteTextures(1, &g_tex);                                                 //Deletes old texture if it exists
    glGenTextures(1, &g_tex);                                                               //Generate texture
    glBindTexture(GL_TEXTURE_2D, g_tex);                                                    //Bind texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);                      //Sets filtering to GL_NEAREST (pixelated, no smoothing)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);                      //Pixelated, no smoothing
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);                                                  //Allocates empty texture memory (nullptr data)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);    //GL_RGB8 format stores 8-bit RGB
}

void DrawTextureFullScreen(int winW, int winH, int texW, int texH) {                        //Prepares orthographic projection for 2D rendering
    glViewport(0, 0, winW, winH);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);                                                               //Disables DepthTest (not needed)
    glEnable(GL_TEXTURE_2D);                                                                //Enables 2D texture rendering
    glBindTexture(GL_TEXTURE_2D, g_tex);                                                    //Binds our raytraced texture

    glBegin(GL_QUADS);                                                                      //Draws a fullscreen quad with texture mapped
    glTexCoord2f(0, 1); glVertex2f(-1, -1);                                                 //Texture coordinates are flipped vertically to match window coordinates
    glTexCoord2f(1, 1); glVertex2f(1, -1);
    glTexCoord2f(1, 0); glVertex2f(1, 1);
    glTexCoord2f(0, 0); glVertex2f(-1, 1);
    glEnd();

    glDisable(GL_TEXTURE_2D);
}

//Win32 boilerplate
static Camera* g_cam = nullptr;
static bool g_drag = false;
static int g_lastX = 0, g_lastY = 0;
static bool g_middleDrag = false;
static int g_midLastX = 0, g_midLastY = 0;

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {               //Handles mouse events for rotating, panning, zooming the camera
    switch (msg) {                                                                          //Uses mouse buttons and wheel to update the global camera g_cam
    case WM_LBUTTONDOWN: {
        g_drag = true;
        SetCapture(hwnd);
        g_lastX = LOWORD(lParam);
        g_lastY = HIWORD(lParam);
    } return 0;

    case WM_LBUTTONUP: {
        g_drag = false;
        ReleaseCapture();
    } return 0;

    case WM_MBUTTONDOWN: {
        g_middleDrag = true;
        SetCapture(hwnd);
        g_midLastX = LOWORD(lParam);
        g_midLastY = HIWORD(lParam);
    } return 0;

    case WM_MBUTTONUP: {
        g_middleDrag = false;
        ReleaseCapture();
    } return 0;

    case WM_MOUSEMOVE: {
        int x = LOWORD(lParam);
        int y = HIWORD(lParam);

        if (g_drag && g_cam) {
            float dx = (x - g_lastX) * 0.005f;
            float dy = (y - g_lastY) * 0.005f;
            g_cam->updateOrbit(dx, -dy);
        }

        if (g_middleDrag && g_cam) {
            float dx = (x - g_midLastX);
            float dy = (y - g_midLastY);

            Vec3 right = normalize(cross(g_cam->forward, g_cam->up));
            Vec3 up = g_cam->up;

            float panSpeed = 0.005f * g_cam->radius;

            Vec3 delta = right * (-dx * panSpeed) + up * (dy * panSpeed);

            g_cam->target = g_cam->target + delta;
            g_cam->updateOrbit(0.0f, 0.0f);

            g_midLastX = x;
            g_midLastY = y;
        }

        g_lastX = x;
        g_lastY = y;
    } return 0;

    case WM_MOUSEWHEEL: {
        if (g_cam) {
            short delta = GET_WHEEL_DELTA_WPARAM(wParam);
            float step = -delta * 0.0015f * g_cam->radius;
            g_cam->zoom(step);
        }
    } return 0;

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hwnd, msg, wParam, lParam);
}

bool SetupGL(HWND hwnd) {                                                           //Obtains device context from window
    g_hDC = GetDC(hwnd);                                                            //Chooses pixel format supporting OpenGL and double buffering
    PIXELFORMATDESCRIPTOR pfd = { 0 };                                              //Creates OpenGL rendering context
    pfd.nSize = sizeof(pfd);                                                        //Makes context current on the thread
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 16;
    pfd.iLayerType = PFD_MAIN_PLANE;
    int pf = ChoosePixelFormat(g_hDC, &pfd);
    if (!pf) return false;
    if (!SetPixelFormat(g_hDC, pf, &pfd)) return false;
    g_glRC = wglCreateContext(g_hDC);
    if (!g_glRC) return false;
    if (!wglMakeCurrent(g_hDC, g_glRC)) return false;
    return true;
}
#pragma endregion

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    //Register                                                                                                              //Registers a window class with custom window procedure
    WNDCLASS wc = {};                                                                                                       //for input/events and own device context for OpenGL 
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = L"RaytraceGLWnd";
    RegisterClass(&wc);

    //OpenGL Context Creation
    int winW = WIDTH; int winH = HEIGHT;                                                                                    //Creates window with extra (+16/+39) pixels to account for borders and title bar
    HWND hwnd = CreateWindowEx(0, wc.lpszClassName, L"CPU Raytracer (OpenGL Display)", WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT, winW + 16, winH + 39, nullptr, nullptr, hInstance, nullptr);
    if (!hwnd) return -1;

    if (!SetupGL(hwnd)) { MessageBoxA(hwnd, "Failed to create GL context", "Error", MB_OK); return -1; }                    //Initializes OpenGL context, shows error message if fails

    //Disable VSync
    typedef BOOL(WINAPI* PFNWGLSWAPINTERVALEXTPROC)(int interval);                                                          //Uses OpenGL extension to disable vertical sync, allows for fastest performance
    PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT = nullptr;
    wglSwapIntervalEXT = (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");
    if (wglSwapIntervalEXT) {
        wglSwapIntervalEXT(0); // 0 to disable VSync, 1 to enable
    }

    //Create LOWRES Texture
    CreateTexture(LOW_WIDTH, LOW_HEIGHT);                                                                                   //Creates a low res texture for faster raytracing

    //Pixel Buffer
    std::vector<unsigned char> pixels(LOW_WIDTH * LOW_HEIGHT * 3);                                                          //Allocates CPU-side pixel buffer for the texture data

    //Sample Scene of spheres
    std::vector<Sphere> scene;                                                                                              //Add 3 spheres to the scene with positions and radii
    scene.emplace_back(Vec3(0.0f, -1.0f, -2.1f), 0.9f);
    scene.emplace_back(Vec3(1.2f, -0.35f, -2.0f), 0.7f);
    scene.emplace_back(Vec3(-1.4f, -1.3f, -2.5f), 0.5f);

    //Camera
    Camera cam(Vec3(0, 0.5, 1), Vec3(0, 0, -2), 60.0f, (float)WIDTH / HEIGHT);                                               //Creates a camera at 0,0.5,3 position and 60degree FOV
    g_cam = &cam;                                                                                                           //Sets global pointer for input handling

    //Water Level
    float waterLevel = 0.1f;
    unsigned long int waterAnimFrame = 0;

    //FPS Tracking
    unsigned long int frameCount = 0;                                                                                       //Tracks framecount and time for FPS tracking
    auto startTime = std::chrono::high_resolution_clock::now();

    bool done = false;
    MSG msg;
    while (!done) {
        //Poll Messages
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) { done = true; break; }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        if (done) break;

        //Animate water level
        waterLevel = 0.1f * sinf(waterAnimFrame++ * 0.1f) + 0.1;

        //Raytrace the scene into pixels                                                                                    //Raytraces for each ray in the LOW RES texture from Camera POV
        for (int y = 0; y < LOW_HEIGHT; ++y) {
            for (int x = 0; x < LOW_WIDTH; ++x) {
                Ray r = cam.getRay(x, y, LOW_WIDTH, LOW_HEIGHT);
                unsigned char* dst = &pixels[(y * LOW_WIDTH + x) * 3];
                marchRay(dst, r, scene, waterLevel, waterAnimFrame);                                                                                    //Marches Ray forward
            }
        }

        //Upload to GL texture
        glBindTexture(GL_TEXTURE_2D, g_tex);                                                                                //Assigns the new frame to the texture
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, LOW_WIDTH, LOW_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

        //Draw
        RECT rct; GetClientRect(hwnd, &rct);                                                                                //Draws the texture to the screen
        int clientW = rct.right - rct.left;
        int clientH = rct.bottom - rct.top;
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        DrawTextureFullScreen(clientW, clientH, LOW_WIDTH, LOW_HEIGHT);
        SwapBuffers(g_hDC);

        //FPS Counting
        frameCount++;                                                                                                       //Calculates the FPS for the last second and assigns it to title bar
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed >= 1.0) {
            double fps = frameCount / elapsed;
            char title[128];
            sprintf_s(title, "CPU Raytracer - FPS: %.2f", fps);
            SetWindowTextA(hwnd, title);
            frameCount = 0;
            startTime = now;
        }
    }

    //Cleanup
    if (g_glRC) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(g_glRC); }                                             //Cleans up the OpenGL context
    if (g_hDC && hwnd) ReleaseDC(hwnd, g_hDC);  
    return 0;                                                                                                               //End of Program
}