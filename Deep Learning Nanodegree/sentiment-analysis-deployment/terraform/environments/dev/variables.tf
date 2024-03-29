variable "jupyter_instance_name" {
  type = "string"
  description = "AWS SageMaker Notebook instance name"
}

variable "jupyter_instance_role_arn" {
  type = "string"
  description = "IAM role to apply upon notebook instance"
}

variable "jupyter_instance_type" {
  type = "string"
  default = "ml.t2.medium"
  description = "AWS SageMaker Notebook instance type"
}