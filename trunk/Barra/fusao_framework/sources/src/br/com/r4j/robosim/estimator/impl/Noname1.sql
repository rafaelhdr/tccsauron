ECB_SAP_TIPO_PROJ

delete FROM NIKU.ODF_CUSTOM_ATTRIBUTES
where 
    object_name = 'project' and
    internal_name = 'ecb_sap_prj_type';
delete FROM NIKU.ODF_VIEW_ATTRIBUTES
where 
attribute_code = 'ecb_sap_prj_type';
ALTER TABLE odf_ca_project
 DROP COLUMN ecb_sap_prj_type;
commit;


CREATE OR REPLACE TRIGGER niku.ecb_sap_createsapid
BEFORE INSERT  OR UPDATE
ON niku.odf_ca_project
REFERENCING NEW AS NEW OLD AS OLD
FOR EACH ROW
declare
	unitId number;
	centroCode varchar(100);
	uniqueName varchar(100);
	sapCode varchar(100);

begin

	if :old.ecb_sap_cria_sap_id is NULL then

		select
			count(assoc.unit_id)
		into
			unitId
		from
			prj_obs_associations assoc
				join prj_obs_units unit
					join prj_obs_types obstypes
					on unit.type_id = obstypes.id and obstypes.name = ecbSapGetUnObsName()
				on unit.id = assoc.unit_id and
				assoc.record_id = :new.id and
				assoc.table_name = 'SRM_PROJECTS';

		if unitId > 0 then
			select
				assoc.unit_id
			into
				unitId
			from
				prj_obs_associations assoc
					join prj_obs_units unit
						join prj_obs_types obstypes
						on unit.type_id = obstypes.id and obstypes.name = ecbSapGetUnObsName()
					on unit.id = assoc.unit_id and
					assoc.record_id = :new.id and
					assoc.table_name = 'SRM_PROJECTS';
			select
				unique_name
			into
				uniqueName
			from
				srm_projects
			where
				id = :new.id;

			select
				obs_map.centro_code
			into
				centroCode
			from
				odf_ca_ecb_sap_obs_estr_inv obs_map,
				prj_obs_associations assoc
			where
				assoc.record_id = obs_map.id and
				assoc.table_name = 'ecb_sap_obs_estr_inv' and
				assoc.unit_id = unitId;

			sapCode := 'PJ-' || centroCode || uniqueName;

			:new.ecb_sap_external_id := sapCode;


			insert into odf_locked_attributes
				(id,
				object_code,
				odf_pk,
				attribute_code,
				created_date,
				created_by,
				last_updated_date,
				last_updated_by)
			values
				(odf_locked_attributes_s1.nextval,
				'project',
				:new.id,
				'ecb_sap_external_id',
				current_date,
				1,
				current_date,
				1);

			insert into odf_locked_attributes
				(id,
				object_code,
				odf_pk,
				attribute_code,
				created_date,
				created_by,
				last_updated_date,
				last_updated_by)
			values
				(odf_locked_attributes_s1.nextval,
				'project',
				:new.id,
				'ecb_sap_cria_sap_id',
				current_date,
				1,
				current_date,
				1);

		end if;
	end if;


end ecb_sap_createSAPId;








CREATE OR REPLACE TRIGGER niku.ecb_sap_createsapid_updt_v2
BEFORE  DELETE  OR UPDATE
ON niku.prj_obs_associations
REFERENCING NEW AS NEW OLD AS OLD
FOR EACH ROW
declare
	ja_tem_associacao EXCEPTION;
	obsCount number;
	prjType number;
	unitId number;
	centroCode varchar(100);
	uniqueName varchar(100);
	sapCode varchar(100);

begin

	select
		count(*)
	into
		obsCount
	from
		prj_obs_units unit
			join prj_obs_types obstypes
			on unit.type_id = obstypes.id and obstypes.name = ecbSapGetUnObsName();

	-- se é da OBS de PN
	if obsCount > 0 then
		raise ja_tem_associacao;
	end if;
end;








CREATE OR REPLACE TRIGGER niku.ecb_sap_createsapid_v2
AFTER INSERT
ON niku.prj_obs_associations
REFERENCING NEW AS NEW OLD AS OLD
FOR EACH ROW
declare
	ja_tem_associacao EXCEPTION;
	obsCount number;
	prjType number;
	centroCode varchar(100);
	uniqueName varchar(100);
	sapCode varchar(100);

begin

	select
		count(*)
	into
		obsCount
	from
		prj_obs_units unit
			join prj_obs_types obstypes
			on unit.type_id = obstypes.id and obstypes.name = ecbSapGetUnObsName();

	-- se é da OBS de PN
	if obsCount > 0 then
		select
			ecb_sap_prj_type
		into
			prjType
		from
			odf_ca_project
		where
			id = :new.record_id;

		-- se é projeto
		if prjType = 1 then

			select
				unique_name
			into
				uniqueName
			from
				srm_projects
			where
				id = :new.record_id;

			select
				obs_map.centro_code
			into
				centroCode
			from
				odf_ca_ecb_sap_obs_estr_inv obs_map
			where
				obs_map.unit_id = :new.unit_id;

			sapCode := 'PJ-' || centroCode || uniqueName;

			update odf_ca_project set
				ecb_sap_external_id = sapCode
			where
				id = :new.record_id;

			insert into odf_locked_attributes
				(id,
				object_code,
				odf_pk,
				attribute_code,
				created_date,
				created_by,
				last_updated_date,
				last_updated_by)
			values
				(odf_locked_attributes_s1.nextval,
				'project',
				:new.record_id,
				'ecb_sap_external_id',
				current_date,
				1,
				current_date,
				1);

			insert into odf_locked_attributes
				(id,
				object_code,
				odf_pk,
				attribute_code,
				created_date,
				created_by,
				last_updated_date,
				last_updated_by)
			values
				(odf_locked_attributes_s1.nextval,
				'project',
				:new.record_id,
				'ecb_sap_cria_sap_id',
				current_date,
				1,
				current_date,
				1);
		end if;
	end if;

end;
