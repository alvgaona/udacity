terraform {
  backend "remote" {
    organization = "alvgaona"

    workspaces {
      name = "sagemaker-deployment"
    }
  }
}

module "jupyter" {
  source = "../../modules/jupyter"

  name = var.jupyter_instance_name
  role_arn = var.jupyter_instance_role_arn
  instance_type = var.jupyter_instance_type
}