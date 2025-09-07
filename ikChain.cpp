// IMPT NOTE: The rest of the code in this file has been omitted as it was not written by me.

// Applies FABRIK algorithm for IK chain
// Bone positions are already calculated by animation file
// Perform customized animation to bones affected by IK
void IKChain::DoAnim(std::vector<BoneTransform>& bones)
{
    if (nullptr == mTargetObj)
        return;

    // convert boneTransforms to model pose matrix
    std::vector<Matrix4> outPoses;
    Animation::BonesToGlobalPose(bones, outPoses, mSkeleton);

#if DEBUG_DRAW == 1
    // --- DEBUG DRAWS ---
    // Draw axis for mCharacter and mTargetObj
    Matrix4 characterMat = mCharacter->GetSkeleton()->GetGlobalInvBindPoses()[0];
    Matrix4 targetObjMat = mTargetObj->mConstants.c_modelToWorld;
    DrawAxis(characterMat);
    DrawAxis(targetObjMat);
    // Draw yellow debug line
    Graphics::Get()->Debug_Line(characterMat.GetTranslation(), targetObjMat.GetTranslation(), 
        Graphics::Color4(1.0f, 1.0f, 0.0f));

    // Draw axis of each joint in IK chain
    for (int i = 0; i < mBoneIndices.size(); i++) {
        DrawAxis(outPoses[mBoneIndices[i]]);
    }
#endif

    // --- FABRIK ---
    size_t endIndex = mBoneIndices.size() - 1;
    std::vector<Matrix4> ikPoses;
    ikPoses = outPoses;
    mBoneDistances.clear();

    // Put end effector onto target
    ikPoses[mBoneIndices[endIndex]] = mTargetObj->mConstants.c_modelToWorld;

    // For each next joint, move to the correct distance and direction away
    for (int k = endIndex - 1; k >= 0; k--) {

        Vector3 nextJoint = ikPoses[mBoneIndices[k + 1]].GetTranslation();
        // Move to correct distance away
        float distance = (outPoses[mBoneIndices[k + 1]].GetTranslation() - outPoses[mBoneIndices[k]].GetTranslation()).Length();
        mBoneDistances.push_back(distance); // add distance between bones to vector for use later
        // Maintain direction
        Vector3 direction = nextJoint - outPoses[mBoneIndices[k]].GetTranslation();
        direction.Normalize();
        Vector3 newPos = nextJoint - (distance * direction);
        // Calculate joint's new position
        ikPoses[mBoneIndices[k]] = Matrix4::CreateTranslation(newPos);
    }

    // Put root back to end
    ikPoses[mBoneIndices[0]] = outPoses[mBoneIndices[0]];

    // For each joint, move to correct distance and direction away
    for (int j = 1; j <= endIndex; j++) {
        Vector3 prevJoint = ikPoses[mBoneIndices[j - 1]].GetTranslation();
        // Maintain direction
        Vector3 direction = prevJoint - ikPoses[mBoneIndices[j]].GetTranslation();
        direction.Normalize();
        // Calculate joint's new position based on previously saved distance and new direction
        Vector3 newPos = prevJoint - (direction * mBoneDistances[endIndex - j]);
        ikPoses[mBoneIndices[j]] = Matrix4::CreateTranslation(newPos);

        // Draw debug line for each IK chain joint
        Graphics::Get()->Debug_Line(prevJoint, newPos, Graphics::Color4(1.0f, 1.0f, 0.0f));
    }

    // --- Matching Character Animation to IK ---
    // For root joint, use outPoses for joint-to-model matrix (driven by character animation)
    for (int k = 0; k < endIndex; k++) {
        int joint = mBoneIndices[k];
        int parent = mSkeleton->GetBone(joint).mParent;
        int child = mBoneIndices[k + 1];

        // For each other joint, use the joint's bind pose to calculate joint-to-parent matrix
        Matrix4 jointToParent = mSkeleton->GetBone(joint).mBindPose.ToMatrix();
        Matrix4 modelToParent = outPoses[parent];
        modelToParent.Invert();
        
        // Then put into IK pose (calculate angle that joint must rotate to go to desired pose)
        // Cross product to get axis
        Vector3 bindPose = mSkeleton->GetBone(child).mBindPose.mPos; // joint space
        bindPose = Matrix4::Transform(bindPose, jointToParent, 0.0f);
        bindPose.Normalize();
        Vector3 ikPose = ikPoses[child].GetTranslation() - ikPoses[joint].GetTranslation(); // model space
        ikPose = Matrix4::Transform(ikPose, modelToParent, 0.0f); // convert to parent space
        ikPose.Normalize();
        Vector3 axis = Vector3::Cross(bindPose, ikPose);
        axis.Normalize();

        // Get angle
        float dot = Vector3::Dot(bindPose, ikPose);
        float angle = acos(dot);

        // Concatenate bind pose rotation with IK rotation to get actual rotation for joint
        const Quaternion ikPoseQuat = Quaternion(axis, angle);
        const Quaternion bindPoseQuat = mSkeleton->GetBone(mBoneIndices[k]).mBindPose.mRot; // joint space
        Quaternion finalRotQuat = Quaternion::Concatenate(bindPoseQuat, ikPoseQuat);

        // Update bone rotation
        bones[joint].mRot = finalRotQuat;

        // Combine with parent-to-model matrix from previous joint to get new joint-to-model matrix
        outPoses[joint] = bones[joint].ToMatrix() * outPoses[parent];
    }
}