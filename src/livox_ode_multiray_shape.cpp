//
// Created by lfc on 2021/2/28.
//

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODETypes.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include <gazebo/physics/ode/ODEMultiRayShape.hh>
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include <ignition/math6/ignition/math.hh>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
LivoxOdeMultiRayShape::LivoxOdeMultiRayShape(CollisionPtr _parent)
    : MultiRayShape(_parent)
{
    this->SetName("ODE Multiray Shape");

    // Create a space to contain the ray space
    this->superSpaceId = dSimpleSpaceCreate(0);

    // Create a space to contain all the rays
    this->raySpaceId = dSimpleSpaceCreate(this->superSpaceId);

    // Set collision bits
    dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
    dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);

    // These three lines may be unessecary
    ODELinkPtr pLink =
        boost::static_pointer_cast<ODELink>(this->collisionParent->GetLink());
    pLink->SetSpaceId(this->raySpaceId);
    boost::static_pointer_cast<ODECollision>(this->collisionParent)->SetSpaceId(
        this->raySpaceId);
}

//////////////////////////////////////////////////
LivoxOdeMultiRayShape::~LivoxOdeMultiRayShape()
{
    dSpaceSetCleanup(this->raySpaceId, 0);
    dSpaceDestroy(this->raySpaceId);

    dSpaceSetCleanup(this->superSpaceId, 0);
    dSpaceDestroy(this->superSpaceId);
}

//////////////////////////////////////////////////
void LivoxOdeMultiRayShape::UpdateRays()
{
    ODEPhysicsPtr ode = boost::dynamic_pointer_cast<ODEPhysics>(
        this->GetWorld()->Physics());

    if (ode == NULL)
        gzthrow("Invalid physics engine. Must use ODE.");

    // Do we need to lock the physics engine here? YES!
    // especially when spawning models with sensors
    {
        boost::recursive_mutex::scoped_lock lock(*ode->GetPhysicsUpdateMutex());

        // Do collision detection
        dSpaceCollide2((dGeomID) (this->superSpaceId),
                       (dGeomID) (ode->GetSpaceId()),
                       this, &UpdateCallback);
    }
}

//////////////////////////////////////////////////
void LivoxOdeMultiRayShape::UpdateCallback(void *_data, dGeomID _o1, dGeomID _o2)
{
    dGeomID ray, object;
    dContactGeom contact;
    LivoxOdeMultiRayShape *self = NULL;

    self = static_cast<LivoxOdeMultiRayShape*>(_data);

    // Check space
    if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
    {
        if (dGeomGetSpace(_o1) == self->superSpaceId ||
            dGeomGetSpace(_o2) == self->superSpaceId)
            dSpaceCollide2(_o1, _o2, self, &UpdateCallback);

        if (dGeomGetSpace(_o1) == self->raySpaceId ||
            dGeomGetSpace(_o2) == self->raySpaceId)
            dSpaceCollide2(_o1, _o2, self, &UpdateCallback);
    }
    else
    {
        ODECollision *collision1 = NULL;
        ODECollision *collision2 = NULL;

        // Get pointers to the underlying collisions
        if (dGeomGetClass(_o1) == dGeomTransformClass)
        {
            collision1 = static_cast<ODECollision*>(
                dGeomGetData(dGeomTransformGetGeom(_o1)));
        }
        else
            collision1 = static_cast<ODECollision*>(dGeomGetData(_o1));

        if (dGeomGetClass(_o2) == dGeomTransformClass)
        {
            collision2 =
                static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o2)));
        }
        else
        {
            collision2 = static_cast<ODECollision*>(dGeomGetData(_o2));
        }

        GZ_ASSERT(collision1, "collision1 is null");
        GZ_ASSERT(collision2, "collision2 is null");

        ODECollision *rayCollision = NULL;
        ODECollision *hitCollision = NULL;

        // Figure out which one is a ray; note that this assumes
        // that the ODE dRayClass is used *soley* by the RayCollision.
        if (dGeomGetClass(_o1) == dRayClass)
        {
            ray = _o1; 
            object = _o2;
            rayCollision = static_cast<ODECollision*>(collision1);
            hitCollision = static_cast<ODECollision*>(collision2);
        }        
        else if (dGeomGetClass(_o2) == dRayClass)
        {
            ray = _o2; 
            object = _o1;
            rayCollision = static_cast<ODECollision*>(collision2);
            hitCollision = static_cast<ODECollision*>(collision1);
        }
        dGeomRaySetParams(ray, 0, 0);
        dGeomRaySetClosestHit(ray, 1);

        // Check for ray/collision intersections
        if (rayCollision && hitCollision)
        {
            // Calling dCollide between a ray and another geom will result in 
            //  at most one contact point. Rays have their own conventions for 
            //  the contact information in the dContactGeom structure (thus it 
            //  is not useful to create contact joints from this information):
            //  * `pos`     - This is the point at which the ray intersects the 
            //                  surface of the other geom, regardless of whether 
            //                  the ray starts from inside or outside the geom.
            //  * `normal`  - This is the surface normal of the other geom at 
            //                  the contact point. if dCollide is passed the ray 
            //                  as its first geom then the normal will be oriented 
            //                  correctly for ray reflection from that surface 
            //                  (otherwise it will have the opposite sign).
            //  * `depth`   - This is the distance from the start of the ray to
            //                  the contact point.
            // (source: wiki)[http://ode.org/wikiold/htmlfile17.html]
            int n = dCollide(object, ray, 1, &contact, sizeof(contact));

            if (n > 0)
            {
                RayShapePtr shape = boost::static_pointer_cast<RayShape>(
                    rayCollision->GetShape());
                if (contact.depth < shape->GetLength())
                {
                    // set ray reflection, made up of diffuse + specular 
                    // assumption: lidar's internal signal processing removes distance + ambiant light effects 
                    auto _sdf = hitCollision->GetSDF();
                    double diff = _sdf->Get<double>("laser_retro"); 
                    double spec = _sdf->Get<double>("laser_spec"); 
                    if (spec > 0) {
                        auto axis = (shape->End() - shape->Start()).Normalized();
                        double dot = contact.normal[0]*axis[0] + 
                                     contact.normal[1]*axis[1] + 
                                     contact.normal[2]*axis[2];
                        spec *= dot; 
                    }
                    shape->SetRetro(diff + spec);

                    // set ray length
                    shape->SetLength(contact.depth);
                }
            }
        }
    }
}

//////////////////////////////////////////////////
void LivoxOdeMultiRayShape::AddRay(const ignition::math::Vector3d &_start,
                              const ignition::math::Vector3d &_end)
{
    MultiRayShape::AddRay(_start, _end);

    ODECollisionPtr odeCollision(new ODECollision(
        this->collisionParent->GetLink()));
    odeCollision->SetName("ode_ray_collision");
    odeCollision->SetSpaceId(this->raySpaceId);

    ODERayShapePtr ray(new ODERayShape(odeCollision));
    odeCollision->SetShape(ray);

    ray->SetPoints(_start, _end);
    this->rays.push_back(ray);
}

