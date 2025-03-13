# Swarmfarm-Headland-Python

The swarmfarm function to generate headlands, called in an AWS Lambda.

Because this lambda uses python 3.8, the code in this repo run in a docker container.
## Run Dev Container
```
./dev.bash
```
This can both run the headlands code with its python 3.8 deployments and run the tests with matplotlib visualisation.

## Deploy to Beta
From inside the container run:
```
./package.bash
```
Then from outside run:
```
./upload_lambda.bash
```
If we are not on a tagged release, it will upload only to $LATEST (aliased to 'beta') and not make a version.

## Release
Merge PR and make a tagged release on Github. Checkout main.
Then, follow same steps as [Deploy to Beta](#deploy-to-beta). If we are on a tagged release, the upload script will prompt options to make a version and alias it to 'prod'.