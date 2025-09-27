#include<bits/stdc++.h>
#include<graphics.h>
#include<thread>
#include<mutex>
#include<future>
using namespace std;

const int W=800,H=450,SPS=360,RFM=10;
random_device rd;
mt19937 gen(rd()); 
uniform_real_distribution<float> randf(0.0f, 1.0f);

// 互斥锁，确保绘图操作的线程安全
mutex mtx;

class Color
{
public:
	float r,g,b;
	Color(){r=g=b=0;}
	Color(float _r, float _g, float _b) : r(_r), g(_g), b(_b) {}
	
	color_t to_col()
	{
		int _r=min(max((int)r,0),255);
		int _g=min(max((int)g,0),255);
		int _b=min(max((int)b,0),255);
		return EGERGB(_r,_g,_b);
	}
	
	Color operator /=(float x)
	{
		r/=x; g/=x; b/=x;
		return *this;
	}
	
	Color operator +=(const Color& x)
	{
		r += x.r; g += x.g; b += x.b;
		return *this;
	}
	
	Color operator *(float factor) const
	{
		return Color(r * factor, g * factor, b * factor);
	}
	
	Color operator +(Color x) const
	{
		return Color(r + x.r, g + x.g, b + x.b);
	}
};

class Vec {
public:
	float x, y, z;
	Vec(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
	
	Vec operator-(const Vec& other) const {
		return Vec(x - other.x, y - other.y, z - other.z);
	}
	
	Vec operator+(const Vec& other) const {
		return Vec(x + other.x, y + other.y, z + other.z);
	}
	
	Vec operator*(float s) const {
		return Vec(x * s, y * s, z * s);
	}
	
	Vec cross(const Vec& other) const {
		return Vec(
			y * other.z - z * other.y,
			z * other.x - x * other.z,
			x * other.y - y * other.x
			);
	}
	
	float length(){
		return sqrt(x*x + y*y + z*z);
	}
	
	Vec normalize() {
		float len = length();
		return Vec(x/len, y/len, z/len);
	}
	
	float dot(const Vec& other) const {
		return x * other.x + y * other.y + z * other.z;
	}
};

class Object
{
public:
	bool reflection;
	Color material;
	virtual float intersect(Vec start, Vec dir) = 0;
	virtual Vec getNormal(Vec hitPoint) = 0;
	~Object() = default;
};

class Round:public Object
{
public:
	Vec pos;
	float radius;
	Round(Vec p, float r, Color c, bool refl = false) {
		pos = p; radius = r; material = c; reflection = refl;
	}
	
	float intersect(Vec start, Vec dir) override
	{
		Vec oc = start - pos;
		
		if(oc.length() <= radius) return 0;
		
		float a = dir.dot(dir);
		float b = 2.0f * oc.dot(dir);
		float c = oc.dot(oc) - radius * radius;
		
		float discriminant = b * b - 4 * a * c;
		
		if (discriminant < 0) return numeric_limits<float>::infinity(); 
		
		float t1 = (-b - sqrt(discriminant)) / (2 * a);
		float t2 = (-b + sqrt(discriminant)) / (2 * a);
		
		if (t1 > 0) return t1;
		if (t2 > 0) return t2;
		
		return numeric_limits<float>::infinity();
	}
	
	Vec getNormal(Vec hitPoint) override {
		return (hitPoint - pos).normalize();
	}
};

class Line : public Object {
public:
	Vec st, ed;
	Line(Vec s, Vec e, Color c, bool refl = false) { 
		st = s; ed = e; material = c; reflection = refl;
	}
	
	float intersect(Vec start, Vec dir) override
	{
		Vec seg_dir = ed - st;
		Vec start_to_st = st - start;
		
		Vec cross_prod = dir.cross(seg_dir);
		float cross_len = cross_prod.dot(cross_prod);
		
		if (fabs(cross_len) < 1e-8)
			return numeric_limits<float>::infinity();
		
		float t = start_to_st.cross(seg_dir).dot(cross_prod) / cross_len;
		float s = start_to_st.cross(dir).dot(cross_prod) / cross_len;
		
		if (t >= -1e-8 && s >= -1e-8 && s <= 1.0 + 1e-8)
			return max(t, 0.0f);
		
		return numeric_limits<float>::infinity();
	}
	
	Vec getNormal(Vec hitPoint) override {
		Vec dir = (ed - st).normalize();
		return Vec(-dir.y, dir.x, 0).normalize();
	}
};

vector<unique_ptr<Object>> Scene;

Vec reflect(Vec incident, Vec normal) {
	return incident - normal * (2 * incident.dot(normal));
}

Color Trace(Vec start, Vec dir, int depth) 
{
	if (depth <= 0) return Color();
	
	float tNear = numeric_limits<float>::infinity();
	Color hitColor;
	Object* hitObj = nullptr;
	
	for (const auto& obj : Scene) 
	{
		float dist = obj->intersect(start, dir);
		if (dist < tNear) 
		{
			tNear = dist;
			hitColor = obj->material;
			hitObj = obj.get();
		}
		
	}
	
	if (tNear == numeric_limits<float>::infinity()) {
		return Color(0, 0, 0);
	}
	
	Vec hitPoint = start + dir * tNear;
	
	if (!hitObj->reflection)return hitColor;
	
	Vec normal = hitObj->getNormal(hitPoint);
	Vec reflectedDir = reflect(dir, normal);
	
	Vec newStart = hitPoint + reflectedDir * 0.01f;
	
	Color reflectedColor = Trace(newStart, reflectedDir, depth - 1);
	
	return hitColor + reflectedColor ;
}

void RenderThread(int startY, int endY)
{
	mt19937 threadGen(rd() + startY);
	uniform_real_distribution<float> threadRandf(0.0f, 1.0f);
	
	for(int y = startY; y < endY; y++) 
	{
		for(int x = 0; x < W; x++) 
		{
			Color sum;
			for(int i = 0; i < SPS; i++) 
			{
				float angle = (i + threadRandf(threadGen)) * 2.0f * PI / SPS;
				sum += Trace(Vec(x, y), Vec(sin(angle), cos(angle)), RFM);
			}
			sum /= SPS;
			
			lock_guard<mutex> lock(mtx);
			putpixel(x, y, sum.to_col());
		}
	}
}

void Render()
{
	const int numThreads = thread::hardware_concurrency();
	vector<thread> threads;
	
	int rowsPerThread = H / numThreads;
	int remainingRows = H % numThreads;
	
	int currentY = 0;
	for(int i = 0; i < numThreads; i++)
	{
		int rows = rowsPerThread + (i < remainingRows ? 1 : 0);
		threads.emplace_back(RenderThread, currentY, currentY + rows);
		currentY += rows;
	}
	
	for(auto& t : threads)
	{
		t.join();
	}
}

void Build()
{
	Scene.push_back(make_unique<Round>(Vec(W/2-200, H/2), 100, Color(255, 0, 255), true));
	Scene.push_back(make_unique<Round>(Vec(W/2+200, H/2), 100, Color(255, 255, 0), true));
	Scene.push_back(make_unique<Round>(Vec(W/2, H/2-100), 70, Color(0, 255, 255), true));
	Scene.push_back(make_unique<Line>(Vec(350, H-200), Vec(W-350, H-200), Color(0, 0, 0), false));
	Scene.push_back(make_unique<Line>(Vec(W/2, H-150), Vec(W/2+100, H-50), Color(255, 255, 255), false));	
	Scene.push_back(make_unique<Line>(Vec(W/2, H-150), Vec(W/2-100, H-50), Color(255, 255, 255), false));	
	Scene.push_back(make_unique<Line>( Vec(W/2-100, H-50),Vec(W/2+100, H-50), Color(255, 255, 255), false));	
}

int main()
{
	initgraph(W, H);
	Build();
	
	auto startTime = chrono::high_resolution_clock::now();
	
	Render();
	
	auto endTime = chrono::high_resolution_clock::now();
	chrono::duration<double> elapsed = endTime - startTime;
	cout << "渲染完成，耗时: " << elapsed.count() << " 秒" << endl;
	
	getch();
	closegraph();
	return 0;
}

