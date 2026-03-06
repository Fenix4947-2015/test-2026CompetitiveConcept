# 2026CompetitiveConcept

This repository contains the code used for the WestCoast Products 2026 [Competitive Concept](https://wcproducts.com/pages/wcp-competitive-concepts).

The project is based on one of CTRE's [Phoenix 6 example projects](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithChoreo). It uses WPILib [command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html) to manage robot subsystems and actions, a [Limelight](https://limelightvision.io/) for vision, and [Choreo](https://choreo.autos/) for autonomous path following.

# Fenix 4947

## À faire

- Réfléchir aux bonnes conditions pour reseed pose. Actuellement c'est beaucoup d'erreur entre l'estimé vision et odométrie et au moins 2 tags. Problème c'est que je suis pas certain que le robot va voir 2 tags avant de commencer son mode autonome. L'autre potentiel problème c'est que je veux pas que ça reset sans raison.

## À tester

### Direction jostick selon aliance

```java
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     * <p>
     * This does not change the robot pose, which is in the
     * {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective.
     * As a result, the robot pose may need to be reset using {@link #resetPose}.
     *
     * @param fieldDirection Heading indicating which direction is forward from
     *                       the {@link SwerveRequest.ForwardPerspectiveValue#BlueAlliance} perspective
     */
    public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
```