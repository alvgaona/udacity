resource "aws_sagemaker_notebook_instance" "juypter" {
  name          = var.name
  role_arn      = var.role_arn
  instance_type = var.instance_type
}