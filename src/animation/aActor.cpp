#include "aActor.h"
#include<iostream> 
#include<algorithm> 

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor()
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	delete m_IKController;
	delete m_BVHController;
	delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode())
		return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	// 2.	Set the y component of the guide position to 0
	// 3.	Set the global rotation of the guide joint towards the guideTarget

	AJoint* root = m_pSkeleton->getRootNode();
	vec3 rootPos = m_Guide.getLocal2Global() * root->getGlobalTranslation();
	vec3 guidePos = m_Guide.getGlobalTranslation();
	m_Guide.setGlobalTranslation(vec3(rootPos[0], 0, rootPos[2]));

	vec3 targetLook = (guideTargetPos - guidePos).Normalize();
	vec3 zAxis_tar = targetLook;
	vec3 yAxis_tar = vec3(0, 1, 0);
	vec3 xAxis_tar = yAxis_tar.Cross(zAxis_tar);
	mat3 rotMat_target = mat3(xAxis_tar, yAxis_tar, zAxis_tar).Transpose();

	m_Guide.setGlobalRotation(rotMat_target);
	m_pSkeleton->update();
}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height
	vec3 root_locTrans = m_pSkeleton->getRootNode()->getLocalTranslation();
	double root_leftHeight = root_locTrans[1] + leftHeight;
	double root_rightHeight = root_locTrans[1] + rightHeight;
	root_locTrans = vec3(root_locTrans[0], std::min(root_leftHeight, root_rightHeight), root_locTrans[2]);
	m_pSkeleton->getRootNode()->setLocalTranslation(root_locTrans);
	m_pSkeleton->update();

	// 2.	Update the charter with Limb-based IK 

	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal

		vec3 yAxis = leftNormal;
		vec3 xAxis = leftFoot->getGlobalRotation()[0];
		vec3 zAxis = yAxis.Cross(xAxis);
		mat3 rotMat = mat3(xAxis, yAxis, zAxis).Transpose();
		leftFoot->setGlobalRotation(rotMat);

		ATarget lTarget;
		vec3 lTarget_gPos = leftFoot->getGlobalTranslation();
		lTarget.setGlobalTranslation(vec3(lTarget_gPos[0], lTarget_gPos[1] + leftHeight, lTarget_gPos[2]));
		m_IKController->IKSolver_Limb(m_IKController->mLfootID, lTarget);

	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal

		vec3 yAxis = rightNormal;
		vec3 xAxis = rightFoot->getGlobalRotation()[0];
		vec3 zAxis = yAxis.Cross(xAxis);
		mat3 rotMat = mat3(xAxis, yAxis, zAxis).Transpose();
		rightFoot->setGlobalRotation(rotMat);

		ATarget rTarget;
		vec3 rTarget_gPos = rightFoot->getGlobalTranslation();
		rTarget.setGlobalTranslation(vec3(rTarget_gPos[0], rTarget_gPos[1] + rightHeight, rTarget_gPos[2]));
		m_IKController->IKSolver_Limb(m_IKController->mRfootID, rTarget);
	}
	m_pSkeleton->update();
}
