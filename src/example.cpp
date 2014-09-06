#include "oxygine-framework.h"
#include "Box2D/Box2D.h"
#include "Box2DDebugDraw.h"
#include <Windows.h>
#include <myo/myo.hpp>

using namespace oxygine;

//it is our resources
//in real project you would have more than one Resources declarations. It is important on mobile devices with limited memory and you would load/unload them
Resources gameResources;

class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
		: onArm(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
	{
	}
	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* mio, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
	}
	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* mio, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		float pitch = asin(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()));
		float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
		// Convert the floating point angles in radians to a scale from 0 to 20.
		roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
	}
	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* mio, uint64_t timestamp, myo::Pose pose)
	{
		currentPose = pose;
		// Vibrate the Myo whenever we've detected that the user has made a fist.
		if (pose == myo::Pose::fist) {
			mio->vibrate(myo::Myo::vibrationShort);
			mio->vibrate(myo::Myo::vibrationShort);
		}
		else if (pose == myo::Pose::waveIn) {
			roll_w = 0;
			pitch_w = 0;
			yaw_w = 0;
			mio->vibrate(myo::Myo::vibrationLong);
		}
	}
	// onArmRecognized() is called whenever Myo has recognized a setup gesture after someone has put it on their
	// arm. This lets Myo know which arm it's on and which way it's facing.
	void onArmRecognized(myo::Myo* mio, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
	{
		onArm = true;
		whichArm = arm;
	}
	// onArmLost() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
	// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
	// when Myo is moved around on the arm.
	void onArmLost(myo::Myo* mio, uint64_t timestamp)
	{
		onArm = false;
	}
	// There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
	// For this example, the functions overridden above are sufficient.
	// We define this function to print the current values that were updated by the on...() functions above.
	void print()
	{
		// Clear the current line
		std::cout << '\r';
		// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
		std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
			<< '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
			<< '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
		if (onArm) {
			// Print out the currently recognized pose and which arm Myo is being worn on.
			// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
			// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
			// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
			std::string poseString = currentPose.toString();
			std::cout << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
				<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
		}
		else {
			// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
			std::cout << "[?]" << '[' << std::string(14, ' ') << ']';
		}
		std::cout << std::flush;
	}

	int getRoll() {
		return roll_w;
	}
	int getPitch() {
		return pitch_w;
	}
	int getYaw() {
		return yaw_w;
	}

	// These values are set by onArmRecognized() and onArmLost() above.
	bool onArm;
	myo::Arm whichArm;
	// These values are set by onOrientationData() and onPose() above.
	int roll_w, pitch_w, yaw_w;
	myo::Pose currentPose;
};


myo::Hub hub("com.example.hello-myo");
DataCollector collector;
myo::Myo* mio;

//sfml
//DECLARE_SMART is helper, it does forward declaration and declares intrusive_ptr typedef for your class
DECLARE_SMART(MainActor, spMainActor);


const float SCALE = 100.0f;
b2Vec2 convert(const Vector2 &pos)
{
	return b2Vec2(pos.x / SCALE, pos.y / SCALE);
}

Vector2 convert(const b2Vec2 &pos)
{
	return Vector2(pos.x * SCALE, pos.y * SCALE);
}


DECLARE_SMART(Circle, spCircle);
class Circle : public Sprite
{
public:
	Circle(b2World *world, const Vector2 &pos, float scale = 1)
	{
		setResAnim(gameResources.getResAnim("circle"));
		setAnchor(Vector2(0.5f, 0.5f));
		setTouchChildrenEnabled(false);

		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = convert(pos);

		b2Body *body = world->CreateBody(&bodyDef);

		setUserData(body);

		setScale(scale);

		b2CircleShape shape;
		shape.m_radius = getWidth() / SCALE / 2 * scale;

		b2FixtureDef fixtureDef;
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.3f;

		body->CreateFixture(&fixtureDef);
		body->SetUserData(this);
	}
};

DECLARE_SMART(Static, spStatic);
class Static : public Box9Sprite
{
public:
	Static(b2World *world, const RectF &rc)
	{
		//setHorizontalMode(Box9Sprite::TILING_FULL);
		//setVerticalMode(Box9Sprite::TILING_FULL);
		setResAnim(gameResources.getResAnim("pen"));
		setSize(rc.getSize());
		setPosition(rc.getLeftTop());
		setAnchor(Vector2(0.5f, 0.5f));

		b2BodyDef groundBodyDef;
		groundBodyDef.position = convert(getPosition());

		b2Body* groundBody = world->CreateBody(&groundBodyDef);

		b2PolygonShape groundBox;
		b2Vec2 sz = convert(getSize()/2);
		groundBox.SetAsBox(sz.x, sz.y);
		groundBody->CreateFixture(&groundBox, 0.0f);
	}
};

class MainActor: public Actor
{
public:	
	b2World *_world;
	spBox2DDraw _debugDraw;

	MainActor():_world(0)
	{	
		setSize(getStage()->getSize());

		spButton btn = new Button;
		btn->setResAnim(gameResources.getResAnim("button"));
		btn->setX(getWidth() - btn->getWidth() - 3);
		btn->setY(3);
		btn->attachTo(this);
		btn->addEventListener(TouchEvent::CLICK, CLOSURE(this, &MainActor::showHideDebug));

		addEventListener(TouchEvent::CLICK, CLOSURE(this, &MainActor::click));


		_world = new b2World(b2Vec2(0, 10), false);


		spStatic ground = new Static(_world, RectF(getWidth() / 2, getHeight() - 10, getWidth() - 100, 30));
		addChild(ground);

		spCircle circle = new Circle(_world, getSize()/2, 2);
		addChild(circle);
	}
	
	void doUpdate(const UpdateState &us)
	{
		//in real project you should make steps with fixed dt, check box2d documentation
		_world->Step(us.dt / 1000.0f, 6, 2);

		// myo
		hub.run(1000 / 20);
		//collector.print();

		//update each body position on display
		b2Body *body = _world->GetBodyList();
		while(body)
		{
			Actor *actor = (Actor *)body->GetUserData();
			b2Body *next = body->GetNext();
			if (actor)
			{
				const b2Vec2& pos = body->GetPosition();
				//actor->setPosition(convert(pos));
				//MessageBoxA(NULL, "string", "title", MB_OK);
				actor->setPosition(Vector2((float)(collector.getYaw() * 50), (float)(collector.getPitch() * 50)));
				//actor->setRotation(body->GetAngle());
				actor->setRotation((float)(collector.getPitch() * 30));

				//remove fallen bodies
				if (actor->getY() > getHeight() + 50)
				{
					body->SetUserData(0);
					_world->DestroyBody(body);

					actor->detach();					
				}
			}			

			body = next;
		}
	}

	void showHideDebug(Event *event)
	{
		TouchEvent *te = safeCast<TouchEvent*>(event);
		te->stopsImmediatePropagation = true;
		if (_debugDraw)
		{
			_debugDraw->detach();
			_debugDraw = 0;
			return;
		}

		_debugDraw = new Box2DDraw;		
		_debugDraw->SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit | b2Draw::e_pairBit | b2Draw::e_centerOfMassBit);
		_debugDraw->attachTo(this);
		_debugDraw->setWorld(SCALE, _world);
		_debugDraw->setPriority(1);
	}

	void click(Event *event)
	{
		TouchEvent *te = safeCast<TouchEvent*>(event);
				
		if (event->target.get() == this)
		{
			spCircle circle = new Circle(_world, te->localPosition);
			circle->attachTo(this);
		}

		if (event->target->getUserData())
		{
			//shot to circle
			std::cout << "hello";
		
			//MessageBoxA(NULL, "string", "title", MB_OK);

			spActor actor = safeSpCast<Actor>(event->target);
			b2Body *body = (b2Body *)actor->getUserData();

			Vector2 dir = actor->getPosition() - te->localPosition;
			dir = dir / dir.length() * body->GetMass() * 500;

			body->ApplyForceToCenter(b2Vec2(dir.x, dir.y));

			spSprite sprite = new Sprite();
			sprite->setResAnim(gameResources.getResAnim("shot"));
			Vector2 local = actor->global2local(te->localPosition);
			sprite->setPosition(local);
			sprite->setAnchor(Vector2(0.5f, 0.5f));
			sprite->attachTo(actor);
		}
	}
};

void example_preinit()
{
}

void example_init()
{
	//load xml file with resources definition
	gameResources.loadXML("res.xml");

	//lets create our client code simple actor
	//prefix 'sp' here means it is intrusive Smart Pointer
	//it would be deleted automatically when you lost ref to it	
	spMainActor actor = new MainActor;
	//and add it to Stage as child
	getStage()->addChild(actor);


	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {
		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.

		std::cout << "Attempting to find a Myo..." << std::endl;
		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForAnyMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		mio = hub.waitForMyo(10000);
		// If waitForAnyMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!mio) {
			throw std::runtime_error("Unable to find a Myo!");
		}
		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
		MessageBoxA(NULL, "found myo", "boom", MB_OK);

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);
		// Finally we enter our main loop.
		//while (1) {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.

			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.
			
		//}
		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
	}
}

void example_destroy()
{
	gameResources.free();
}

void example_update()
{

}

