#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->spinBox->setRange(0,255);
    ui->spinBox_2->setRange(0,255);
    ui->spinBox_3->setRange(0,255);

    ui->spinBox_5->setRange(-100,100);
    ui->spinBox_6->setRange(-100,100);
    ui->spinBox_7->setRange(-100,100);

    ui->label->setPixmap(QPixmap::fromImage(*paper));
}

MainWindow::~MainWindow()
{
    delete ui;
}

#include <fstream>
#include <cmath>
#include <new>

struct Scene;
struct Object;
struct HitInfo;
struct Color;
struct Raytracer;
struct IMaterial;
struct Phong;
void clamp255(Color& col);

struct Vec3
{
    double x,y,z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    Vec3 operator + (const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator - (const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator - () const { return Vec3(-x, -y, -z); }
    Vec3 operator * (double d) const { return Vec3(x*d, y*d, z*d); }
    Vec3 operator / (const Vec3& v) { return Vec3(x/v.x, y/v.y, z/v.z); }
    Vec3 operator / (double d) const { return Vec3(x/d, y/d, z/d); }

    double lenght() const
    {
      return sqrt(x*x + y*y + z*z);
    }
    Vec3 normalize() const
    {
    double mg = lenght();
    return Vec3(x/mg,y/mg,z/mg);
    }
    double lenghtPow() const
    {
      return (x*x+y*y+z*z);
    }

    Vec3 cross(Vec3 v)
    {
    return Vec3((y * v.z) - (z * v.y),
               (z * v.x) - (x * v.z),
               (x * v.y) - (y * v.x));
    }
};

inline double dot(const Vec3& a, const Vec3& b)
{
    return (a.x*b.x + a.y*b.y + a.z*b.z);
}

Vec3 reflect(Vec3 vec, Vec3 normal)
{
      double d_dot = dot(normal,vec);
      return normal * d_dot * 2 - vec;
}

const Vec3 emptyVec3 = Vec3(0,0,0);

struct Vec2
{
  double x,y;
  Vec2(double x = 0, double y = 0) : x(x), y(y) {}
  Vec2 operator + (const Vec2& v) const { return Vec2(x+v.x, y+v.y); }
  Vec2 operator - (const Vec2& v) const { return Vec2(x-v.x, y-v.y); }
  Vec2 operator * (double d) const { return Vec2(x*d, y*d); }
  Vec2 operator / (double d) const { return Vec2(x/d, y/d); }
};



struct Ray
{
    Vec3 o,d;
    double epsilon = 0.000001;
    double max = 1.0/0.0;
    Ray(const Vec3& o, const Vec3& d) : o(o), d(d.normalize()) {}
};

struct Color
{
    double red;
    double green;
    double blue;
    Color(double red, double green, double blue) : red(red), green(green), blue(blue){}
    Color operator + (Color d){ return Color(red+d.red,green+d.green,blue+d.blue);}
    Color operator * (Color d){ return Color(red*d.red,green*d.green,blue*d.blue);}
    Color operator * (double t){ return Color(red*t,green*t,blue*t);}
    Color operator / (double t){ return Color(red/t,green/t,blue/t);}
    double getRed(){ return red*255; }
    double getGreen(){ return green*255; }
    double getBlue(){ return blue*255; }
};

const Color white = Color(255/255.0,255/255.0,255/255.0);
const Color black = Color(0,0,0);
const Color red = Color(255/255.0,0,0);
const Color green = Color(0,255/255.0,0);
const Color blue = Color(0,0,255/255.0);
const Color yellow = Color(255/255.0,255/255.0,0);
const Color cyan = Color(0,255/255.0,255/255.0);
const Color magenta = Color(255/255.0,0,255/255.0);
const Color gray = Color(192/255.0,192/255.0,192/255.0);

struct PointLight
{
    Vec3 position = emptyVec3;
    Color color = white;
    PointLight(Vec3 position, Color color)
    {
        this->position = position;
        this->color = color;
    }
    Vec3 getPosition() const
    {
        return position;
    }
    Color getColor() const
    {
        return color;
    }
};




struct ICamera
{
    virtual Ray GetRayTo(Vec2 relativeLocation) = 0;
    virtual ~ICamera(){}
};

struct OrthonormalBasis
{
    Vec3 u = emptyVec3;
    Vec3 v = emptyVec3;
    Vec3 w = emptyVec3;
    OrthonormalBasis(Vec3 eye, Vec3 lookAt, Vec3 up)
    {
        w = eye - lookAt;
        w = w.normalize();
        u = up.cross(w);
        u = u.normalize();
        v = w.cross(u);
    }
};

inline Vec3 operator * (OrthonormalBasis onb, Vec3 v)
{
    return (onb.u * v.x + onb.v * v.y + onb.w * v.z);
}

struct Triangle
{
    Vec3 a;
    Vec3 b;
    Vec3 c;
    Triangle(Vec3 a = emptyVec3, Vec3 b = emptyVec3, Vec3 c = emptyVec3) : a(a), b(b), c(c){}
};

bool rayIntersectsTriangle(Ray ray, Triangle inTriangle, double& outIntersectionPoint, Vec3& normal)
{
    double EPSILON = 0.00000000001;

    Vec3 vertex0 = inTriangle.a;
    Vec3 vertex1 = inTriangle.b;
    Vec3 vertex2 = inTriangle.c;

    Vec3 edge1, edge2, h, s, q;

    double a,f,u,v;

    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;

    h = ray.d.cross(edge2);
    a = dot(edge1,h);

    if (a > -EPSILON && a < EPSILON)
    {
       return false;    // This ray is parallel to this triangle.
    }

    f = 1.0/a;
    s = ray.o - vertex0;
    u = f * (dot(s,h));

    if (u < 0.0 || u > 1.0)
    {
        return false;
    }

    q = s.cross(edge1);
    v = f * dot(ray.d,q);
    if (v < 0.0 || (u + v) > 1.0)
    {
        return false;
    }
    // At this stage we can compute t to find out where the intersection point is on the line.
    double t = f * dot(edge2,q);
    if (t > EPSILON) // ray intersection
    {
        outIntersectionPoint = t;
        normal = (edge1.cross(edge2)).normalize();
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
    {
        return false;
    }
}

struct Orthogonal : public ICamera
{
    Vec3 EyePosition;
    double Angle;
    Vec2 CameraSize;
    Orthogonal(Vec3 eye, double angle, Vec2 size) :
              EyePosition(eye),
              Angle(angle),
              CameraSize(size){}

    Ray GetRayTo(Vec2 pictureLocation)
    {
     Vec3 direction = Vec3(sin(Angle),0,cos(Angle));
     direction = direction.normalize();
     Vec2 offsetFromCenter = Vec2(pictureLocation.x * CameraSize.x,pictureLocation.y * CameraSize.y);
     Vec3 position = Vec3(
            EyePosition.x + offsetFromCenter.x * cos(Angle),
            EyePosition.y + offsetFromCenter.y,
            EyePosition.z + offsetFromCenter.x * sin(Angle));
     return Ray(position,direction);
    }
};

struct Pinhole : public ICamera
{
    OrthonormalBasis onb = OrthonormalBasis(emptyVec3,emptyVec3,emptyVec3);
    Vec3 origin = emptyVec3;
    double distance;
    Pinhole(Vec3 origin, Vec3 lookAt,Vec3 up, double distance)
    {
        this->onb = OrthonormalBasis(origin, lookAt, up);
        this->origin = origin;
        this->distance = distance;
    }
    Ray GetRayTo(Vec2 relativeLocation)
    {
        return Ray(origin, RayDirection(relativeLocation));
    }
    Vec3 RayDirection(Vec2 v)
    {
        return onb * Vec3(v.x, v.y, -distance);
    }
};


struct HitInfo
{
    Object* hitObject;
    Scene* scene;
    int depth; // max ilosc odbic
    Vec3 normal = emptyVec3;
    Vec3 hitPoint = emptyVec3;
    Ray ray = Ray(emptyVec3,emptyVec3);
};

/*struct ISampleGenerator
{
    virtual Vec2* Sample(int count) = 0;
    virtual ~ISampleGenerator();
};

struct Jittered : ISampleGenerator
{
    Jittered(int seed)
    {
        srand(seed);
    }

    Vec2* Sample(int count)
    {
        int sampleRow = int(sqrt(count));
        Vec2* result = new Vec2[sampleRow * sampleRow];
        for (int x = 0; x < sampleRow; x++)
        {
            for (int y = 0; y < sampleRow; y++)
            {
                double fracX = (x + (double(rand())/RAND_MAX)) / sampleRow;
                double fracY = (y + (double(rand())/RAND_MAX)) / sampleRow;
                *(result+(x * sampleRow + y)) = Vec2(fracX, fracY);
            }
        }
    return result;
    }
};

struct ISampleDistributor
{
   virtual Vec2 MapSample(Vec2 sample) = 0;
   virtual ~ISampleDistributor();
};

struct SquareDistributor : ISampleDistributor
{
    Vec2 MapSample(Vec2 sample){ return sample; }
};*/

struct IMaterial
{
   virtual Color shade(Raytracer* tracer, HitInfo hit) = 0;
};

struct Box
{
    Vec3 minV;
    Vec3 maxV;
    Box(Vec3 minV, Vec3 maxV) : minV(minV), maxV(maxV) {}
    Box() : minV(emptyVec3), maxV(emptyVec3) {}
};

struct Object
{
    Box BVH;
    Object(IMaterial* material):material(material){}
    Object(IMaterial* material, QVector <Triangle> triangles):material(material),triangles(triangles){}
    IMaterial* material;
    QVector <Triangle> triangles;
    // Repair!
    void setVecMinAndMax()
    {
        double minX = triangles[0].a.x;
        for(int i = 0; i < triangles.size(); ++i)
        {
            if(triangles[i].a.x < minX)
            {
                minX = triangles[i].a.x;
            }
            if(triangles[i].b.x < minX)
            {
                minX = triangles[i].b.x;
            }
            if(triangles[i].c.x < minX)
            {
                minX = triangles[i].c.x;
            }
        }
        double minY = triangles[0].a.y;
        for(int i = 0; i < triangles.size(); ++i)
        {
            if(triangles[i].a.y < minY)
            {
                minY = triangles[i].a.y;
            }
            if(triangles[i].b.y < minY)
            {
                minY = triangles[i].b.y;
            }
            if(triangles[i].c.y < minY)
            {
                minY = triangles[i].c.y;
            }
        }
        double minZ = triangles[0].a.z;
        for(int i = 0; i < triangles.size(); ++i)
        {
            if(triangles[i].a.z < minZ)
            {
                minZ = triangles[i].a.z;
            }
            if(triangles[i].b.z < minZ)
            {
                minZ = triangles[i].b.z;
            }
            if(triangles[i].c.z < minZ)
            {
                minZ = triangles[i].c.z;
            }
        }

        double maxX = triangles[0].a.x;
        for(int i = 0; i < triangles.size(); ++i)
        {
            if(triangles[i].a.x > maxX)
            {
                maxX = triangles[i].a.x;
            }
            if(triangles[i].b.x > maxX)
            {
                maxX = triangles[i].b.x;
            }
            if(triangles[i].c.x > maxX)
            {
                maxX = triangles[i].c.x;
            }
        }
        double maxY = triangles[0].a.y;
        for(int i = 0; i < triangles.size(); ++i)
        {
            if(triangles[i].a.y > maxY)
            {
                maxY = triangles[i].a.y;
            }
            if(triangles[i].b.y > maxY)
            {
                maxY = triangles[i].b.y;
            }
            if(triangles[i].c.y > maxY)
            {
                maxY = triangles[i].c.y;
            }
        }
        double maxZ = triangles[0].a.z;
        for(int i = 0; i < triangles.size(); ++i)
        {
            if(triangles[i].a.z > maxZ)
            {
                maxZ = triangles[i].a.z;
            }
            if(triangles[i].b.z > maxZ)
            {
                maxZ = triangles[i].b.z;
            }
            if(triangles[i].c.z > maxZ)
            {
                maxZ = triangles[i].c.z;
            }
        }
        BVH = Box(Vec3(minX,minY,minZ),Vec3(maxX,maxY,maxZ));
    }

    bool hit(Ray ray)
    {
        double tmin = (BVH.minV.x - ray.o.x) / ray.d.x;
        double tmax = (BVH.maxV.x - ray.o.x) / ray.d.x;

        if (tmin > tmax)
        {
            double buf = tmax;
            tmax = tmin;
            tmin = buf;
        }

        double tymin = (BVH.minV.y - ray.o.y) / ray.d.y;
        double tymax = (BVH.maxV.y - ray.o.y) / ray.d.y;

        if (tymin > tymax)
        {
            double buf = tymax;
            tymax = tymin;
            tymin = buf;
        }

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        if (tymin > tmin)
            tmin = tymin;

        if (tymax < tmax)
            tmax = tymax;

        double tzmin = (BVH.minV.z - ray.o.z) / ray.d.z;
        double tzmax = (BVH.maxV.z - ray.o.z) / ray.d.z;

        if (tzmin > tzmax)
        {
            double buf = tzmax;
            tzmax = tzmin;
            tzmin = buf;
        }

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        if (tzmin > tmin)
            tmin = tzmin;

        if (tzmax < tmax)
            tmax = tzmax;

        return true;
    }
    IMaterial* getMaterial() { return material; }
    ~Object(){}
};

struct Scene
{
    Color background;
    QVector <Object*> objects;
    QVector <PointLight*> lights;

    Scene(Color background = black) : background(background) {}

    void addObj(Object* obj)
    {
        objects.push_back(obj);
    }

    void addLight(PointLight* light)
    {
        lights.push_back(light);
    }

    HitInfo traceRay(const Ray& ray)
    {
        HitInfo result = HitInfo();
        Vec3 normal = emptyVec3;
        double minimalDistance = ray.max; // najbliższe trafienie
        double hitDistance = 0; // zmienna pomocnicza, ostatnia odległość

        for( int i = 0; i < objects.size(); ++i)
        {
            if(objects[i]->hit(ray))
            {
                for(int j = 0; j < objects[i]->triangles.size(); ++j)
                {
                    if (rayIntersectsTriangle(ray,objects[i]->triangles[j],hitDistance,normal) && hitDistance < minimalDistance) // jeśli najbliższe trafienie
                    {
                        minimalDistance = hitDistance; // nowa najmniejsza odległość
                        result.hitObject = objects[i]; // zapisz kolor trafionego obiektu
                        result.normal = normal;
                    }
                }
            }
        }

        if(result.hitObject != NULL)
        {
            result.hitPoint = ray.o + ray.d * minimalDistance;
            result.ray = ray;
            result.scene = this;
        }

        return result;
    }

    bool anyObstacleBetween(Vec3 pointA, Vec3 pointB)
    {
     // odległość od cieniowanego punktu do światła
     Vec3 vectorAB = pointB - pointA;
     double distAB = vectorAB.lenght();
     double currDistance = 1.0/0.0;
     // promień (półprosta) z cieniowanego punktu w kierunku światła
     Ray ray = Ray(pointA, vectorAB);

     Vec3 ignored = emptyVec3;
     for(int i = 0; i < objects.size(); ++i)
     {
     // jeśli jakiś obiekt jest na drodze promienia oraz trafienie
     // nastąpiło bliżej niż odległość punktu do światła,
     // obiekt jest w cieniu
         if(objects[i]->hit(ray))
         {
             for(int j = 0; j < objects[i]->triangles.size(); ++j)
             {
                 if (rayIntersectsTriangle(ray,objects[i]->triangles[j],currDistance,ignored) && currDistance < distAB) // jeśli najbliższe trafienie
                 {
                     return true;
                 }
             }
         }
     }
     // obiekt nie jest w cieniu
     return false;
    }
};


struct PerfectDiffuse : IMaterial
{
    Color materialColor = black;
    PerfectDiffuse(Color materialColor)
    {
        this->materialColor = materialColor;
    }

    Color shade(Raytracer* tracer, HitInfo hit)
    {
        Color totalColor = black;
        for(int i = 0; i < hit.scene->lights.size(); ++i)
        {
            Vec3 inDirection = (hit.scene->lights[i]->position - hit.hitPoint).normalize();
            double diffuseFactor = dot(inDirection,hit.normal);
            if (diffuseFactor < 0) { continue; }
            if (hit.scene->anyObstacleBetween(hit.hitPoint, hit.scene->lights[i]->position))
            {
                continue;
            }
            totalColor = totalColor + hit.scene->lights[i]->color * materialColor * diffuseFactor;
        }
        return totalColor;
    }

};

struct Raytracer : public MainWindow
{
    int maxDepth;

    Raytracer(int maxDepth)
    {
        this->maxDepth = maxDepth;
    }

    void raytrace(Scene* scene, ICamera* camera, int w, int h, QImage* paper)
    {
        uchar* ptr;
        for(int y = 0; y < h; y++)
        {
            ptr = paper->scanLine(y);
            for (int x = 0; x < w; x++)
            {
                // przeskalowanie x i y do zakresu [-1; 1]
                Vec2 pictureCoordinates = Vec2(
                ((x + 0.5) / (double)w) * 2 - 1,
                ((y + 0.5) / (double)h) * 2 - 1);
                // wysłanie promienia i sprawdzenie w co właściwie trafił
                Ray ray = camera->GetRayTo(pictureCoordinates);
                HitInfo info = scene->traceRay(ray);
                // ustawienie odpowiedniego koloru w obrazie wynikowym
                Color color = black;
                if (info.hitObject != NULL)
                {
                    color = shadeRay(scene,ray,0);
                    clamp255(color);
                }
                else
                {
                    color = scene->background ;
                }


                ptr[4*x] = uchar(color.getBlue());      //b
                ptr[4*x+1] = uchar(color.getGreen());   //g
                ptr[4*x+2] = uchar(color.getRed());     //r
            }
            if(y%4==0)
            {
                qDebug() << y/4 + 1 << "%";
            }
        }
    }

    Color shadeRay(Scene* scene, Ray ray, int currentDepth)
    {
        if(currentDepth > maxDepth)
        {
            return black;
        }
        HitInfo info = scene->traceRay(ray);
        info.depth = currentDepth + 1;

        if (info.hitObject == NULL)
        {
            return scene->background;
        }
        IMaterial* material = new PerfectDiffuse(gray);

        return material->shade(this, info);
    }
};


struct Phong : public IMaterial
{
    Color materialColor = black;
    double diffuseCoeff;
    double specular;
    double specularExponent;
    Phong(Color materialColor,double diffuse,double specular,double specularExponent)
    {
        this->materialColor = materialColor;
        this->diffuseCoeff = diffuse;
        this->specular = specular;
        this->specularExponent = specularExponent;
    }

    Color radiance(PointLight light, HitInfo hit)
    {
        Vec3 inDirection = (light.position - hit.hitPoint).normalize();
        double diffuseFactor = dot(inDirection,hit.normal);
        if (diffuseFactor < 0)
        {
            return black;
        }
        Color result = light.color * materialColor * diffuseFactor * diffuseCoeff;
        double phongFactor = PhongFactor(inDirection, hit.normal, -hit.ray.d);
        if (phongFactor != 0.0)
        {
            result = result + materialColor * specular * phongFactor;
        }
        return result;
    }


    Color shade(Raytracer* tracer, HitInfo hit)
    {
        Color totalColor = black;
        for(int i = 0; i < hit.scene->lights.size(); ++i)
        {
            Vec3 inDirection = (hit.scene->lights[i]->position - hit.hitPoint).normalize();
            double diffuseFactor = dot(inDirection,hit.normal);
            if (diffuseFactor < 0)
            {
                continue;
            }
            if (hit.scene->anyObstacleBetween(hit.hitPoint, hit.scene->lights[i]->position))
            {
                continue;
            }

            Color result = hit.scene->lights[i]->color * materialColor * diffuseFactor * diffuseCoeff;
            double phongFactor = PhongFactor(inDirection, hit.normal, -hit.ray.d);
            if (phongFactor != 0.0)
            {
                result = result + materialColor * specular * phongFactor;
            }
            totalColor = totalColor + result;
        }
        return totalColor;
    }


    double PhongFactor(Vec3 inDirection, Vec3 normal, Vec3 toCameraDirection)
    {
        Vec3 reflected = reflect(inDirection, normal);
        double cosAngle = dot(reflected,toCameraDirection);
        if (cosAngle <= 0)
        {
            return 0;
        }
        return pow(cosAngle, specularExponent);
    }

};

struct Reflective : IMaterial
{
    Phong direct = Phong(black,0,0,0); // do bezpośredniego oświetlenia
    double reflectivity;
    Color reflectionColor = black;
    // diffuse - jak mocno rozprasza, specular - polysk, exponent - potega cos, reflectivity - jak mocno odpija swiatko ( =0 => Phong) (ref = 1, diff = 0 => lustro)
    // diffuse + reflectivity <= 1
    Reflective(Color materialColor , double diffuse , double specular , double exponent , double reflectivity)
    {
        this->direct = Phong(materialColor, diffuse, specular, exponent);
        this->reflectivity = reflectivity;
        this->reflectionColor = materialColor;
    }
    Color shade(Raytracer* tracer, HitInfo hit)
    {
        Vec3 toCameraDirection = -hit.ray.d;

        Color radiance = direct.shade(tracer, hit);
        Vec3 reflectionDirection = reflect(toCameraDirection, hit.normal);
        Ray reflectedRay = Ray(hit.hitPoint, reflectionDirection);
        radiance = radiance + tracer->shadeRay(hit.scene, reflectedRay, hit.depth) * reflectionColor * reflectivity;

        return radiance;
    }
};

IMaterial* whiteMir = new Reflective(white, 0.0, 1, 300, 1);
IMaterial* redMir = new Reflective(red, 0.0, 1, 300, 1);
IMaterial* greenMir = new Reflective(green, 0.0, 1, 300, 1);
IMaterial* blueMir = new Reflective(blue, 0.0, 1, 300, 1);
IMaterial* grayMir = new Reflective(gray, 0.0, 1, 300, 1);

IMaterial* whiteMetal = new Phong(white, 0.8, 1, 20);
IMaterial* redMetal = new Phong(red, 0.8, 1, 20);
IMaterial* greenMetal = new Phong(green, 0.8, 1, 20);
IMaterial* blueMetal = new Phong(blue, 0.8, 1, 20);
IMaterial* grayMetal = new Phong(gray, 0.8, 1, 20);

IMaterial* whiteMat = new  PerfectDiffuse(white);
IMaterial* redMat = new  PerfectDiffuse(red);
IMaterial* greenMat = new PerfectDiffuse(green);
IMaterial* blueMat = new PerfectDiffuse(blue);
IMaterial* grayMat = new PerfectDiffuse(gray);

/*struct Sphere : public Object
{
    Vec3 center;
    double r;
    Sphere(Vec3 center, double r, IMaterial* material) : Object(material) , center(center), r(r){}
    bool intersect(const Ray& ray, double &t, Vec3& outNormal)
    {
        Vec3 o = ray.o;
        Vec3 d = ray.d;
        Vec3 oc = o - center; //distance
        const double a = d.lenghtPow();
        const double b = dot(oc, d) * 2;
        const double c = oc.lenghtPow() - r*r;
        double disc = b*b - 4 * c * a;
        if (disc < ray.epsilon)
        {
            return false;
        }
        disc = sqrt(disc);
        double t0 = (-b - disc)/(2*a);
        if(t0 < ray.epsilon)
        {
            t0 = (-b + disc)/(2*a);
            if(t0 < ray.epsilon)
            {
                return false;
            }
        }
        t = t0;
        Vec3 pi = ray.o + ray.d * t;
        outNormal = (pi - center).normalize();
        return true;
    }
    IMaterial* getMaterial()
    {
        return material;
    }
};*/

/*struct Plane : public Object
{
    /// <summary>Punkt przez który płaszczyzna przechodzi</summary>
    Vec3 point = emptyVec3;
    /// <summary>Normalna do płaszczyzny</summary>
    Vec3 normal = emptyVec3;

    Plane(Vec3 point, Vec3 normal, IMaterial* material) : Object(material)
    {
        this->point = point;
        this->normal = normal;
        this->material = material;
    }

    bool intersect(const Ray& ray, double& t, Vec3& outNormal)
    {
        double dis = dot((point - ray.o), normal.normalize()) / dot(ray.d, normal);
        if (dis > ray.epsilon)
        {
            t = dis;
            outNormal = normal;
            return true;
        }
        return false;
    }

    Vec3 getNormal()
    {
        return normal;
    }
    IMaterial* getMaterial()
    {
        return material;
    }
};*/


void clamp255(Color& col)
{
    col.red = (col.red > 1) ? 1 : (col.red < 0) ? 0 : col.red;
    col.green = (col.green > 1) ? 1 : (col.green < 0) ? 0 : col.green;
    col.blue = (col.blue > 1) ? 1 : (col.blue < 0) ? 0 : col.blue;
}

bool loadOBJ(std::string filepath, QVector<Vec3>& vortex, QVector<Vec2>& uvs, QVector<Vec3>& normal)
{

    FILE* file = fopen(filepath.c_str(), "r");
    if( file == NULL )
    {
        qDebug() << "Impossible to open the file !\n";
        return false;
    }

    QVector< unsigned int > vertexIndices, uvIndices, normalIndices;
    QVector< Vec3 > temp_vertices;
    QVector< Vec2 > temp_uvs;
    QVector< Vec3 > temp_normals;

    while( 1 )
    {

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader
        if ( strcmp( lineHeader, "v" ) == 0 )
        {
            Vec3 vertex;
            fscanf(file, "%lf %lf %lf\n", &vertex.x, &vertex.y, &vertex.z );
            temp_vertices.push_back(vertex);
        }
        else if ( strcmp( lineHeader, "vn" ) == 0 )
        {
            Vec3 normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z );
            temp_normals.push_back(normal);
        }
        else if ( strcmp( lineHeader, "f" ) == 0 )
        {
            unsigned int vertexIndex[3];// uvIndex[3], normalIndex[3], ignore1, ignore2, ignore3;
            int matches = fscanf(file, "%d/%d/%d\n",
                                 &vertexIndex[0],// &uvIndex[0], &normalIndex[0],
                                 &vertexIndex[1],// &uvIndex[1], &normalIndex[1],
                                 &vertexIndex[2]);// &uvIndex[2], &normalIndex[2]);
            if (matches != 3)
            {
                qDebug() << matches ;
                qDebug() << "File can't be read by our simple parser : ( Try exporting with other options\n";
                return false;
            }

            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
            //uvIndices    .push_back(uvIndex[0]);
           // uvIndices    .push_back(uvIndex[1]);
           // uvIndices    .push_back(uvIndex[2]);
           // normalIndices.push_back(normalIndex[0]);
            //normalIndices.push_back(normalIndex[1]);
           // normalIndices.push_back(normalIndex[2]);
        }

        for( unsigned int i = 0; i < vertexIndices.size(); i++ )
        {
            unsigned int vertexIndex = vertexIndices[i] - 1;
            vortex.push_back(temp_vertices[ vertexIndex ]);
        }
    }
    fclose(file);
    return true;
}

Scene* scene = new Scene(Color(0,100/255.0,170/255.0));

void MainWindow::raytracing()
{
  scene->addLight(new PointLight(Vec3(4, 3, -5), white));

  ICamera* camera = new Pinhole(Vec3(1, 2, -5),Vec3(0, 0, 0),Vec3(0, -1, 0),1);

  Raytracer* tracer = new Raytracer(1);

  QVector <Vec3> test1;
  QVector <Vec2> test2;
  QVector <Vec3> test3;
  if(!loadOBJ("./obj2/cow.obj",test1,test2,test3))
  {
      qDebug() << "OBJ error";
      return;
  };
  qDebug() << "OBJ loaded";

  QVector <Triangle> vortexs;
  for(int i = 0; i < (test1.size() / 3); ++i)
  {
          vortexs.push_back( Triangle(test1[(i * 3) + 0],
                                      test1[(i * 3) + 1],
                                      test1[(i * 3) + 2]) );
  }

  Object* obj = new Object(grayMat,vortexs);
  obj->setVecMinAndMax();
  scene->addObj(obj);

  tracer->raytrace(scene, camera, W, H, paper);

  ui->label->setPixmap(QPixmap::fromImage(*paper));
}

void MainWindow::on_pushButton_clicked()
{
    raytracing();
    update();
}

/* ------------- presets ----------------
 * Base
 * scene->addObj(new Sphere(Vec3(-4, 0, 0), 2, blueMat));
 * scene->addObj(new Sphere(Vec3(4, 0, 0), 2, greenMetal));
 * scene->addObj(new Sphere(Vec3(0, 0, 3), 2, redMir));
 * scene->addObj(new Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), grayMir));
 *
 * scene->addLight(new PointLight(Vec3(0, 5, -5), white));
 *
 * ICamera* camera = new Pinhole(Vec3(0, 1, -5),Vec3(0, 0, 0),Vec3(0, -1, 0),1);
 *
 * Mirror
 * scene->addObj(new Sphere(Vec3(0, 0, 0), 9, new Reflective(white,0,1,30,1)));
 * scene->addObj(new Sphere(Vec3(-4, 0, 0), 2, new Reflective(Color(51/255.0,1,1),0,1,30,1)));
 * scene->addObj(new Sphere(Vec3(4, 0, 0), 2, new Reflective(Color(1,178/255.0,102/255.0),0,1,30,1)));
 * scene->addObj(new Sphere(Vec3(0, 0, 3), 2, new Reflective(Color(0,153/255.0,76/255.0),0,1,30,1)));
 * scene->addObj(new Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), grayMir));
 *
 * scene->addLight(new PointLight(Vec3(0, 3, -3), white));
 * scene->addLight(new PointLight(Vec3(0, 3, 5), white));
 * scene->addLight(new PointLight(Vec3(3, 3, -3), white));
 * scene->addLight(new PointLight(Vec3(-3, 3, 5), white));
 *
 * ICamera* camera = new Pinhole(Vec3(0, 1, -5),Vec3(0, 0, 0),Vec3(0, -1, 0),1);
*/

void MainWindow::on_pushButton_3_clicked()
{
    scene = new Scene(Color(0,100/255.0,170/255.0));

    ICamera* camera = new Pinhole(Vec3(0, 1, -5),Vec3(0, 0, 0),Vec3(0, -1, 0),1);

    Raytracer* tracer = new Raytracer(1);



}
